"""
lxros Python API
"""

from typing import Optional
import time
import rclpy  # <-- add this

from .context import LxContext, get_default_context
from .node import LxNode


def init_node(
    name: str,
    *,
    namespace: Optional[str] = None,
    context: Optional[LxContext] = None,
    args: Optional[list[str]] = None,
    **kwargs,
) -> LxNode:
    """
    Create an LxNode in the given context (or the default context).

    Typical usage:

        import lxros as lx
        from std_msgs.msg import String

        node = lx.init_node("example")
        node.sub("chatter", String, cb)
        pub = node.pub("chatter2", String)
        lx.spin()
    """
    if context is None:
        ctx = get_default_context()
    else:
        ctx = context
    # 'args' only matters when context is first created; no harm in ignoring here.
    return ctx.create_node(name, namespace=namespace, **kwargs)


def get_context() -> LxContext:
    """Return the default global LxContext."""
    return get_default_context()

def ok() -> bool:
    """
    Return True if the ROS client library is initialized and not shut down.
    """
    return rclpy.ok()

def spin_once(timeout: float = 0.0, context: Optional[LxContext] = None) -> None:
    """
    Process callbacks once, non-blocking if timeout=0.
    """
    ctx = context or get_default_context()
    ctx.spin_once(timeout_sec=timeout)


def sleep(seconds: float) -> None:
    time.sleep(seconds)




def spin(context: Optional[LxContext] = None) -> None:
    """
    Spin the given context, or the default context if none is provided.

    This will process callbacks for all nodes registered in that context.
    """
    ctx = context or get_default_context()
    ctx.spin()


__all__ = [
    "LxContext",
    "LxNode",
    "init_node",
    "get_context",
    "ok",
    "spin_once",
    "sleep",
    "spin",
]
