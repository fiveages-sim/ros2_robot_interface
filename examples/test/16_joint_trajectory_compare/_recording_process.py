"""为关节轨迹对比示例管理 ``record_joint_interfaces.py`` 子进程。"""

from __future__ import annotations

import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable


SCRIPT_DIR = Path(__file__).resolve().parent
RECORDER_SCRIPT = SCRIPT_DIR / "record_joint_interfaces.py"
RECORD_DATA_DIR = SCRIPT_DIR / "record_data"
RECORDER_READY_TIMEOUT_SEC = 5.0
RECORDER_EXIT_TIMEOUT_SEC = 30.0
RECORDER_TERMINATE_TIMEOUT_SEC = 3.0
SESSION_PREFIX_RE = re.compile(r"^[a-z][a-z0-9_]*$")


def _make_session_dir(prefix: str) -> Path:
    if not SESSION_PREFIX_RE.fullmatch(prefix):
        raise ValueError(f"非法录制会话前缀: {prefix!r}")
    RECORD_DATA_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = 1
    while True:
        name = f"{prefix}_{timestamp}" if suffix == 1 else f"{prefix}_{timestamp}_{suffix}"
        session_dir = RECORD_DATA_DIR / name
        try:
            session_dir.mkdir()
            return session_dir
        except FileExistsError:
            suffix += 1


@dataclass
class RecorderProcess:
    """支持先发停止信号、完成机器人清理后再等待绘图落盘。"""

    process: subprocess.Popen
    output_dir: Path
    _stop_requested: bool = False

    def request_stop(self) -> None:
        """非阻塞请求 recorder 正常退出，以触发 CSV close 与 ``--plot``。"""
        if self._stop_requested or self.process.poll() is not None:
            return
        self._stop_requested = True
        try:
            self.process.send_signal(signal.SIGINT)
        except ProcessLookupError:
            # poll() 与 signal 之间子进程正常退出，后续 wait() 仍会回收并检查结果。
            pass

    def _terminate_and_reap(self) -> int:
        """停止没有正常退出的 recorder，并确保回收子进程。"""
        if self.process.poll() is None:
            try:
                self.process.terminate()
            except ProcessLookupError:
                pass
        try:
            return self.process.wait(timeout=RECORDER_TERMINATE_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            print("[recorder] terminate 超时，发送 kill")
            try:
                self.process.kill()
            except ProcessLookupError:
                pass
            return self.process.wait(timeout=RECORDER_TERMINATE_TIMEOUT_SEC)

    def _verify_outputs(self) -> None:
        expected_files = (
            self.output_dir / "joint_interfaces.csv",
            self.output_dir / "joint_interfaces_position.html",
        )
        missing = [
            str(path)
            for path in expected_files
            if not path.is_file() or path.stat().st_size == 0
        ]
        if missing:
            raise RuntimeError(f"recorder 输出缺失或为空: {missing}")

    def wait(self) -> None:
        """等待 recorder 完成绘图；超时后逐级 terminate/kill。"""
        if not self._stop_requested:
            self.request_stop()
        try:
            return_code = self.process.wait(timeout=RECORDER_EXIT_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            print("[recorder] 正常停止超时，发送 terminate")
            return_code = self._terminate_and_reap()
        except BaseException:
            # 子进程位于独立 session，不会随父进程的 Ctrl+C 自动退出。
            # 等待过程被中断时必须先回收子进程，再把中断继续抛给调用方。
            self._terminate_and_reap()
            raise
        if return_code != 0:
            raise RuntimeError(f"recorder 异常退出，returncode={return_code}")
        self._verify_outputs()
        print(f"[recorder] 已结束，数据与图表目录: {self.output_dir}")


def start_recorder(
    session_prefix: str,
    health_check: Callable[[], None] | None = None,
    failure_cleanup: Callable[[], None] | None = None,
) -> RecorderProcess:
    """启动 recorder 并等待 CSV；异常时先执行调用方的安全清理。"""
    if not RECORDER_SCRIPT.is_file():
        raise FileNotFoundError(f"录制脚本不存在: {RECORDER_SCRIPT}")

    output_dir = _make_session_dir(session_prefix)
    command = [
        sys.executable,
        str(RECORDER_SCRIPT),
        "--plot",
        "--output",
        str(output_dir),
    ]
    print(f"[recorder] 启动: {' '.join(command)}")
    process = subprocess.Popen(command, cwd=SCRIPT_DIR, start_new_session=True)
    recorder = RecorderProcess(process=process, output_dir=output_dir)

    try:
        csv_path = output_dir / "joint_interfaces.csv"
        deadline = time.monotonic() + RECORDER_READY_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if health_check is not None:
                health_check()
            return_code = process.poll()
            if return_code is not None:
                raise RuntimeError(
                    f"recorder 启动阶段异常退出，returncode={return_code}"
                )
            # recorder 首次收到 introspection 数据后才写表头并 flush；等待非空文件，
            # 可避免运动开始后才发现 topic 没有数据。
            if csv_path.is_file() and csv_path.stat().st_size > 0:
                print(f"[recorder] 已就绪: {csv_path}")
                return recorder
            time.sleep(0.05)
        raise TimeoutError(
            f"recorder 未在 {RECORDER_READY_TIMEOUT_SEC:.1f}s 内写入首帧数据: {csv_path}"
        )
    except BaseException:
        # Popen 已成功但 recorder 尚未返回给调用方；任何异常（尤其 Ctrl+C）
        # 都必须先让调用方把机器人置于安全状态，再停止并回收 detached 子进程。
        safety_cleanup_failed = False
        if failure_cleanup is not None:
            try:
                failure_cleanup()
            except BaseException as cleanup_exc:
                safety_cleanup_failed = True
                print(f"[recorder] 启动失败后的安全清理结果: {cleanup_exc}")
        try:
            if safety_cleanup_failed:
                # HOLD 未确认时不等待最长 30 秒绘图；快速回收后立刻让 main.finally
                # 重试机器人清理。
                recorder._terminate_and_reap()
            else:
                recorder.request_stop()
                recorder.wait()
        except BaseException as cleanup_exc:
            print(f"[recorder] 启动失败后的子进程清理结果: {cleanup_exc}")
        raise


def finish_recorder(recorder: RecorderProcess | None) -> None:
    """等待 CSV/图表落盘；失败或 Ctrl+C 由调用方决定最终退出状态。"""
    if recorder is None:
        return
    recorder.wait()
