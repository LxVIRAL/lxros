# lxros/actions.py

from __future__ import annotations
from typing import Any, Callable, Optional

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle

class LxActionServer:
    """
    Thin wrapper around rclpy.ActionServer.
    """

    def __init__(
        self,
        node,
        name: str,
        action_type: Any,
        server: ActionServer,
        execute_callback: Callable[[Any], Any],
    ) -> None:
        self._node = node
        self._name = name
        self._type = action_type
        self._server = server
        self._execute_callback = execute_callback

    @property
    def name(self) -> str:
        return self._name

    @property
    def action_type(self) -> Any:
        return self._type

    @property
    def raw(self) -> ActionServer:
        return self._server

    def publish_feedback(self, **fields: Any) -> None:
        fb = self._type.Feedback()
        for k, v in fields.items():
            setattr(fb, k, v)
        self._server.publish_feedback(fb)


class LxActionClient:
    """
    Synchronous wrapper around rclpy.ActionClient.
    """

    def __init__(self, node, name: str, action_type: Any, client: ActionClient) -> None:
        self._node = node
        self._name = name
        self._type = action_type
        self._client = client

    @property
    def name(self) -> str:
        return self._name

    @property
    def action_type(self) -> Any:
        return self._type

    @property
    def raw(self) -> ActionClient:
        return self._client

    def send(
        self,
        goal: Any = None,
        *,
        timeout: Optional[float] = None,
        feedback_cb: Optional[Callable[[Any], None]] = None,
        **fields: Any,
    ) -> Any:
        """
        Synchronous goal send. Blocks until result arrives.
        """

        # Build goal
        if goal is None:
            goal = self._type.Goal()
            for k, v in fields.items():
                setattr(goal, k, v)

        # Wait for server
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f"Action server '{self._name}' not available")

        # Send goal asynchronously
        send_future = self._client.send_goal_async(goal, feedback_callback=feedback_cb)

        from .context import get_default_context

        ctx = self._node._context or get_default_context()

        # Wait for goal handle
        while rclpy.ok() and not send_future.done():
            ctx.spin_once(timeout_sec=0.1)

        if send_future.cancelled():
            raise RuntimeError("Goal send was cancelled")
        if send_future.exception() is not None:
            raise send_future.exception()

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal was rejected")

        # Wait for result
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            ctx.spin_once(timeout_sec=0.1)

        if result_future.cancelled():
            raise RuntimeError("Action result cancelled")
        if result_future.exception() is not None:
            raise result_future.exception()

        return result_future.result().result

    def send_async(self, goal: Any = None, **fields: Any):
        # Build or convert goal
        if goal is None:
            goal = self._type.Goal()
            for k, v in fields.items():
                setattr(goal, k, v)
        elif isinstance(goal, dict):
            g = self._type.Goal()
            for k, v in goal.items():
                setattr(g, k, v)
            goal = g
        else:
            # Assume the user passed an actual Goal message
            pass

        # Wait for server
        if not self._client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError(f"Action server '{self._name}' not available")

        # Send async
        future = self._client.send_goal_async(goal)

        from .context import get_default_context
        ctx = self._node._context or get_default_context()

        # Wait for goal handle
        while rclpy.ok() and not future.done():
            ctx.spin_once(timeout_sec=0.1)

        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        return goal_handle, result_future



    def cancel(self, goal_handle):
        return self._client._cancel_goal_async(goal_handle)
