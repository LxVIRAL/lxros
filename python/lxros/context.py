import sys
import atexit
import threading
from typing import List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor, Executor

from .node import LxNode


class LxContext:
    """
    ROS 2 context wrapper that owns an executor and a set of LxNode instances.
    """

    def __init__(
        self,
        *,
        executor: Optional[Executor] = None,
        args: Optional[list[str]] = None,
    ) -> None:
        if not rclpy.ok():
            rclpy.init(args=args or sys.argv)

        self._executor: Executor = executor or MultiThreadedExecutor()
        self._nodes: List[LxNode] = []
        self._lock = threading.Lock()
        self._shutdown = False

    @property
    def executor(self) -> Executor:
        return self._executor

    def create_node(
        self,
        name: str,
        *,
        namespace: Optional[str] = None,
        **kwargs,
    ) -> LxNode:
        """Create and register a new LxNode in this context."""
        node = LxNode(
            name=name,
            namespace=namespace,
            context=self,
            **kwargs,
        )
        self.register_node(node)
        return node

    def register_node(self, node: LxNode) -> None:
        """Register an existing LxNode with this context and add to executor."""
        with self._lock:
            # Enforce unique node names within this context
            for n in self._nodes:
                if n.name == node.name:
                    raise ValueError(
                        f"Node with name '{node.name}' already exists in this context"
                    )
            self._nodes.append(node)
            self._executor.add_node(node._node)  # access underlying rclpy.Node

    def unregister_node(self, node: LxNode) -> None:
        with self._lock:
            if node in self._nodes:
                self._nodes.remove(node)
                try:
                    self._executor.remove_node(node._node)
                except Exception:
                    # Executor may already be shutting down; ignore.
                    pass

    def spin(self) -> None:
        """Spin this context's executor until shutdown."""
        try:
            self._executor.spin()
        finally:
            # Do not shut rclpy here; that is handled by shutdown()
            pass

    def spin_once(self, timeout_sec: Optional[float] = None) -> None:
        """Spin this context's executor once."""
        self._executor.spin_once(timeout_sec=timeout_sec)

    def shutdown(self) -> None:
        """Shutdown nodes and rclpy (if we own it)."""
        if self._shutdown:
            return
        self._shutdown = True

        with self._lock:
            for n in list(self._nodes):
                try:
                    self._executor.remove_node(n._node)
                except Exception:
                    pass
            self._nodes.clear()

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                # Already shut down or error; ignore.
                pass


# ---- Default global context (singleton) ----

_default_context: Optional[LxContext] = None
_default_context_lock = threading.Lock()


def get_default_context() -> LxContext:
    global _default_context
    if _default_context is None:
        with _default_context_lock:
            if _default_context is None:
                _default_context = LxContext()
    return _default_context


def _shutdown_default_context() -> None:
    global _default_context
    if _default_context is not None:
        _default_context.shutdown()
        _default_context = None


atexit.register(_shutdown_default_context)
