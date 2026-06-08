"""Center-of-mass estimation from URDF XML and joint state positions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping
import xml.etree.ElementTree as ET

import numpy as np

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover - depends on ROS/Pinocchio environment
    pin = None
    _PINOCCHIO_IMPORT_ERROR = exc
else:
    _PINOCCHIO_IMPORT_ERROR = None


class ComEstimatorError(RuntimeError):
    """Raised when CoM estimation cannot be performed safely."""


@dataclass(frozen=True)
class FrameDiagnostics:
    """Relationship between Pinocchio universe, URDF root link, and reported CoM frame."""

    pinocchio_universe_name: str
    urdf_root_link: str
    requested_frame_id: str
    relation: str
    frame_id_matches_root_link: bool


@dataclass(frozen=True)
class ComEstimate:
    """Result of a center-of-mass computation."""

    xyz: tuple[float, float, float]
    frame_id: str
    missing_joints: tuple[str, ...]
    unsupported_joints: tuple[str, ...]
    frame_diagnostics: FrameDiagnostics | None = None


@dataclass(frozen=True)
class MimicJoint:
    """URDF mimic relation for a joint."""

    source_joint: str
    multiplier: float = 1.0
    offset: float = 0.0


class ComEstimator:
    """Build a Pinocchio model from URDF XML and compute CoM from joint positions."""

    def __init__(self, urdf_xml: str, frame_id: str = "base_footprint") -> None:
        if pin is None:
            raise ComEstimatorError(
                "Pinocchio Python is not importable. Source the ROS environment or install Pinocchio Python."
            ) from _PINOCCHIO_IMPORT_ERROR
        if not urdf_xml or not urdf_xml.strip():
            raise ComEstimatorError("URDF XML is empty; cannot build Pinocchio model.")

        self.frame_id = frame_id
        self.model = pin.buildModelFromXML(urdf_xml)
        self.root_link = self._collect_urdf_root_link(urdf_xml)
        universe_name = str(self.model.names[0]) if len(self.model.names) else "universe"
        frame_id_matches_root_link = self.frame_id == self.root_link
        if frame_id_matches_root_link:
            relation = (
                "固定基座 Pinocchio 模型：universe 与 URDF 根连杆坐标系重合；"
                "centerOfMass 返回值与请求帧坐标一致"
            )
        else:
            relation = (
                "固定基座 Pinocchio 模型：centerOfMass 在 universe/URDF 根连杆坐标系下；"
                "请求帧与 URDF 根连杆不同，当前未做坐标变换"
            )
        self.frame_diagnostics = FrameDiagnostics(
            pinocchio_universe_name=universe_name,
            urdf_root_link=self.root_link,
            requested_frame_id=self.frame_id,
            relation=relation,
            frame_id_matches_root_link=frame_id_matches_root_link,
        )
        self.data = self.model.createData()
        self.q_neutral = pin.neutral(self.model)
        self.single_dof_joint_names: tuple[str, ...] = self._collect_single_dof_joint_names()
        self.unsupported_joint_names: tuple[str, ...] = self._collect_unsupported_joint_names()
        self.mimic_joints: dict[str, MimicJoint] = self._collect_mimic_joints(urdf_xml)

    def compute(
        self,
        joint_positions: Mapping[str, float],
        *,
        allow_missing_with_neutral: bool = False,
    ) -> ComEstimate:
        """Compute CoM using joint positions keyed by joint name.

        Args:
            joint_positions: Mapping from joint name to current position.
            allow_missing_with_neutral: If false, missing single-DoF q joints raise
                ComEstimatorError. If true, missing joints keep their neutral q value
                and are reported in the result.
        """
        resolved_joint_positions = self._with_mimic_joint_positions(joint_positions)
        q = np.array(self.q_neutral, copy=True)
        missing: list[str] = []

        for joint_name in self.single_dof_joint_names:
            if joint_name not in resolved_joint_positions:
                missing.append(joint_name)
                continue
            joint_id = self.model.getJointId(joint_name)
            q_index = self.model.idx_qs[joint_id]
            q[q_index] = float(resolved_joint_positions[joint_name])

        if missing and not allow_missing_with_neutral:
            raise ComEstimatorError(
                "Missing joint positions required by Pinocchio q: " + ", ".join(sorted(missing))
            )

        com = pin.centerOfMass(self.model, self.data, q)
        return ComEstimate(
            xyz=(float(com[0]), float(com[1]), float(com[2])),
            frame_id=self.frame_id,
            missing_joints=tuple(sorted(missing)),
            unsupported_joints=self.unsupported_joint_names,
            frame_diagnostics=self.frame_diagnostics,
        )

    def _collect_single_dof_joint_names(self) -> tuple[str, ...]:
        names: list[str] = []
        for joint_id, joint_name in enumerate(self.model.names):
            if joint_id == 0:
                continue
            if self.model.nqs[joint_id] == 1:
                names.append(joint_name)
        return tuple(names)

    def _collect_unsupported_joint_names(self) -> tuple[str, ...]:
        names: list[str] = []
        for joint_id, joint_name in enumerate(self.model.names):
            if joint_id == 0:
                continue
            if self.model.nqs[joint_id] > 1:
                names.append(joint_name)
        return tuple(names)

    @staticmethod
    def _collect_urdf_root_link(urdf_xml: str) -> str:
        try:
            root = ET.fromstring(urdf_xml)
        except ET.ParseError as exc:
            raise ComEstimatorError("URDF XML is invalid; cannot parse root link.") from exc

        links = {
            link.attrib["name"]
            for link in root.findall("link")
            if "name" in link.attrib
        }
        child_links = {
            child.attrib["link"]
            for joint in root.findall("joint")
            for child in [joint.find("child")]
            if child is not None and "link" in child.attrib
        }
        root_links = sorted(links - child_links)
        if len(root_links) != 1:
            raise ComEstimatorError(
                "Expected exactly one URDF root link, got: " + ", ".join(root_links)
            )
        return root_links[0]

    @staticmethod
    def _collect_mimic_joints(urdf_xml: str) -> dict[str, MimicJoint]:
        try:
            root = ET.fromstring(urdf_xml)
        except ET.ParseError as exc:
            raise ComEstimatorError("URDF XML is invalid; cannot parse mimic joints.") from exc

        mimic_joints: dict[str, MimicJoint] = {}
        for joint in root.findall("joint"):
            joint_name = joint.attrib.get("name")
            mimic = joint.find("mimic")
            if not joint_name or mimic is None:
                continue

            source_joint = mimic.attrib.get("joint")
            if not source_joint:
                continue

            mimic_joints[joint_name] = MimicJoint(
                source_joint=source_joint,
                multiplier=float(mimic.attrib.get("multiplier", "1.0")),
                offset=float(mimic.attrib.get("offset", "0.0")),
            )
        return mimic_joints

    def _with_mimic_joint_positions(self, joint_positions: Mapping[str, float]) -> dict[str, float]:
        resolved = {
            str(joint_name): float(position)
            for joint_name, position in joint_positions.items()
        }

        pending = set(self.mimic_joints)
        while pending:
            progressed = False
            for joint_name in tuple(pending):
                if joint_name in resolved:
                    pending.remove(joint_name)
                    progressed = True
                    continue

                mimic = self.mimic_joints[joint_name]
                if mimic.source_joint not in resolved:
                    continue

                resolved[joint_name] = (
                    resolved[mimic.source_joint] * mimic.multiplier + mimic.offset
                )
                pending.remove(joint_name)
                progressed = True

            if not progressed:
                break

        return resolved
