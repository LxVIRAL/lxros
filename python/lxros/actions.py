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

    def send(self, goal: Any = None, timeout: Optional[float] = None, **fields: Any):
        """
        Blocking goal send.
        Mirrors the behavior of LxServiceClient.call().
        """
        goal_future, holder = self.send_async(goal, **fields)

        # Retrieve context
        from .context import get_default_context
        ctx = self._node._context or get_default_context()

        # Wait for goal handle
        start = rclpy.clock.Clock().now()
        while rclpy.ok() and not goal_future.done():
            ctx.spin_once(timeout_sec=0.1)
            if timeout is not None:
                if (rclpy.clock.Clock().now() - start).nanoseconds > timeout * 1e9:
                    raise TimeoutError("Timed out waiting for goal to be accepted")

        goal_handle = goal_future.result()

        # Wait for result_future to be created
        while holder["future"] is None:
            ctx.spin_once(timeout_sec=0.05)

        result_future = holder["future"]

        # Now wait for the result
        while rclpy.ok() and not result_future.done():
            ctx.spin_once(timeout_sec=0.1)

        res = result_future.result()
        if hasattr(res, "result"):
            return res.result
        return None


    def send_async(self, goal: Any = None, **fields: Any):
        """
        Non-blocking goal send.
        Returns:
            goal_future, result_future
        """
        # Build goal
        if goal is None:
            goal = self._type.Goal()
            for k, v in fields.items():
                setattr(goal, k, v)
        elif isinstance(goal, dict):
            g = self._type.Goal()
            for k, v in goal.items():
                setattr(g, k, v)
            goal = g
        # else assume user passed a Goal instance

        # Wait for server
        if not self._client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError(f"Action server '{self._name}' not available")

        # Send goal request (async)
        goal_future = self._client.send_goal_async(goal)

        # This will be filled once the goal handle arrives
        result_future_holder = {"future": None}

        def _goal_response_cb(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                # Reject: create a completed result future
                import concurrent.futures
                dummy = concurrent.futures.Future()
                dummy.set_result(None)
                result_future_holder["future"] = dummy
            else:
                result_future_holder["future"] = goal_handle.get_result_async()

        goal_future.add_done_callback(_goal_response_cb)

        # Do not spin or block here.
        return goal_future, result_future_holder





    def cancel(self, goal_handle):
        return self._client._cancel_goal_async(goal_handle)
