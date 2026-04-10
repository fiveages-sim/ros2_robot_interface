"""
CCS workspace sweep + OCS2 reachability evaluation.

Scan volume (default): absolute coordinates in **arm_base**:
  x in [0.2, 0.9], y in [0.08, 0.75], z in [-0.45, -0.05] m (left arm sweep).

Implementation: ``reachability/scan_support.py`` (all scan helpers), ``runner.py``,
``cli.py``, ``quick_verify.py``. This file is the CLI entry only.

Multi-instance: `export ROS_DOMAIN_ID=N` or **--ros-domain-id N** here.

**Quick verify** (`--quick-verify`): optional orchestration with `examples/fa_sim_launch.py`.
"""

from __future__ import annotations

import sys
from pathlib import Path

# Ensure `import ros2_robot_interface` works even when running this script directly
# (sys.path[0] is `examples/test`, not the repo root).
_repo_root = Path(__file__).resolve().parents[2]
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

_this_dir = Path(__file__).resolve().parent
if str(_this_dir) not in sys.path:
    sys.path.insert(0, str(_this_dir))

from reachability import ReachabilityScanRunner, build_parser, main_quick_verify  # noqa: E402


def run(args):
    """Backward-compatible alias for ``ReachabilityScanRunner().run(args)``."""
    return ReachabilityScanRunner().run(args)


def main() -> int:
    # Defensive check: ensure we imported the local `reachability` package.
    import reachability as _reach  # noqa: E402

    reach_path = str(getattr(_reach, "__file__", ""))
    if reach_path and str(_this_dir) not in reach_path:
        raise RuntimeError(
            "Imported a non-local `reachability` package. "
            f"expected under {str(_this_dir)!r}, got {reach_path!r}. "
            "Fix your PYTHONPATH / environment, or run from the workspace root."
        )

    args = build_parser().parse_args()
    if args.quick_verify != "none":
        return main_quick_verify(args)
    return ReachabilityScanRunner().run(args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        sys.exit(130)
