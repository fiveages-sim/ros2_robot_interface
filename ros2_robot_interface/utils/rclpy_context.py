"""Process-wide rclpy context ownership tracking.

``rclpy.init()`` is global to the process, so a library must not shut the context
down if the host application created it. Everything in this package that needs a
context goes through :func:`ensure_rclpy_context`, which remembers whether *we*
initialized it, and :func:`shutdown_rclpy_if_owned` releases it once the last
interface is gone.
"""

from __future__ import annotations

import logging
import threading

import rclpy

logger = logging.getLogger(__name__)

_LOCK = threading.Lock()
_OWNED = False


def ensure_rclpy_context() -> None:
    """Initialize rclpy if needed, remembering whether this package owns it."""
    global _OWNED
    if rclpy.ok():
        return
    with _LOCK:
        if not rclpy.ok():
            rclpy.init()
            _OWNED = True


def shutdown_rclpy_if_owned() -> None:
    """Shut down the rclpy context created by this package (no-op otherwise).

    Call this after every interface has been disconnected, e.g. during Ctrl+C
    teardown, so DDS participants and rclpy threads are released.
    """
    global _OWNED
    with _LOCK:
        if not _OWNED:
            return
        _OWNED = False
        try:
            rclpy.try_shutdown()
        except Exception as e:  # pragma: no cover - rclpy internal
            logger.debug("rclpy shutdown failed: %s", e)


def context_is_owned() -> bool:
    """Whether this package initialized the current rclpy context."""
    return _OWNED
