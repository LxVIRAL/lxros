import inspect
from typing import Any, Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# Forward-declared type to avoid circular imports in type checking
if False:  # pragma: no cover
    from .context import LxContext  # type: ignore[unused-import]


def _make_qos(qos: Optional[Any], queue_size: int) -> QoSProfile:
    """
    Normalize qos argument to a QoSProfile.
    - None  -> QoSProfile(depth=queue_size)
    - int   -> QoSProfile(depth=qos)
    - QoSProfile -> itself
    """
    if qos is None:
        return QoSProfile(depth=queue_size)
    if isinstance(qos, int):
        return QoSProfile(depth=qos)
    if isinstance(qos, QoSProfile):
        return qos
    raise TypeError(f"Unsupported qos type: {type(qos)}")


class LxPublisher:
    def __init__(self, node: "LxNode", topic: str, msg_type: Any, publisher: Any) -> None:
        self._node = node
        self._topic = topic
        self._msg_type = msg_type
        self._publisher = publisher

    @property
    def topic(self) -> str:
        return self._topic

    @property
    def msg_type(self) -> Any:
        return self._msg_type

    @property
    def raw(self) -> Any:
        """Underlying rclpy.Publisher."""
        return self._publisher

    def publish(self, msg: Any = None, **fields: Any) -> None:
        """
        Publish a message.
        - If 'msg' is provided, publish it directly.
        - Otherwise, construct a message of msg_type from keyword fields.
        """
        if msg is None:
            msg = self._msg_type()
            for k, v in fields.items():
                setattr(msg, k, v)
        self._publisher.publish(msg)


class LxSubscription:
    def __init__(self, node: "LxNode", topic: str, subscription: Any) -> None:
        self._node = node
        self._topic = topic
        self._subscription = subscription

    @property
    def topic(self) -> str:
        return self._topic

    @property
    def raw(self) -> Any:
        """Underlying rclpy.Subscription."""
        return self._subscription


class LxService:
    def __init__(self, node: "LxNode", name: str, service: Any) -> None:
        self._node = node
        self._name = name
        self._service = service

    @property
    def name(self) -> str:
        return self._name

    @property
    def raw(self) -> Any:
        """Underlying rclpy.Service."""
        return self._service


class LxServiceClient:
    def __init__(self, node: "LxNode", name: str, srv_type: Any, client: Any) -> None:
        self._node = node
        self._name = name
        self._srv_type = srv_type
        self._client = client

    @property
    def name(self) -> str:
        return self._name

    @property
    def srv_type(self) -> Any:
        return self._srv_type

    @property
    def raw(self) -> Any:
        """Underlying rclpy.Client."""
        return self._client

    def call(self, request: Any = None, timeout: Optional[float] = None, **fields: Any) -> Any:
        """
        Synchronous service call.

        Usage:
            resp = client.call(a=1, b=2)
        or:
            req = MySrv.Request()
            req.a = 1
            resp = client.call(req)

        This spins the node's context executor until the response arrives or timeout.
        """
        from .context import get_default_context  # local import to avoid cycles

        if request is None:
            request = self._srv_type.Request()
            for k, v in fields.items():
                setattr(request, k, v)

        if not self._client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Service '{self._name}' not available")

        future = self._client.call_async(request)

        ctx = self._node._context or get_default_context()
        # Simple spin loop on this future
        while rclpy.ok() and not future.done():
            ctx.spin_once(timeout_sec=0.1)

        if future.cancelled():
            raise RuntimeError("Service call was cancelled")

        if future.exception() is not None:
            raise future.exception()

        return future.result()


class LxNode:
    """
    Thin wrapper around rclpy.node.Node providing convenience methods.
    """

    def __init__(
        self,
        name: str,
        *,
        context: "LxContext",
        namespace: Optional[str] = None,
        **kwargs: Any,
    ) -> None:
        from .context import LxContext  # avoid hard dependency at module import

        if not isinstance(context, LxContext):
            raise TypeError("context must be an LxContext")

        self._context = context
        self._node: Node = Node(name, namespace=namespace, **kwargs)

        self._publishers: list[LxPublisher] = []
        self._subscriptions: list[LxSubscription] = []
        self._services: list[LxService] = []
        self._timers: list[Any] = []

    # ---- Core properties ----

    @property
    def name(self) -> str:
        return self._node.get_name()

    @property
    def namespace(self) -> str:
        return self._node.get_namespace()

    @property
    def rclpy_node(self) -> Node:
        """Underlying rclpy.Node."""
        return self._node

    @property
    def context(self) -> "LxContext":
        return self._context

    # ---- Logging ----

    @property
    def logger(self):
        return self._node.get_logger()

    def info(self, msg: str, *args: Any) -> None:
        if args:
            msg = msg % args
        self.logger.info(msg)

    def warn(self, msg: str, *args: Any) -> None:
        if args:
            msg = msg % args
        self.logger.warn(msg)

    def error(self, msg: str, *args: Any) -> None:
        if args:
            msg = msg % args
        self.logger.error(msg)

    def debug(self, msg: str, *args: Any) -> None:
        if args:
            msg = msg % args
        self.logger.debug(msg)


    # ---- Publishers ----

    def pub(
        self,
        topic: str,
        msg_type: Any,
        *,
        qos: Optional[Any] = None,
        queue_size: int = 10,
        name: Optional[str] = None,  # reserved for future use
    ) -> LxPublisher:
        qos_profile = _make_qos(qos, queue_size)
        publisher = self._node.create_publisher(msg_type, topic, qos_profile=qos_profile)
        wrapper = LxPublisher(self, topic, msg_type, publisher)
        self._publishers.append(wrapper)
        return wrapper

    # ---- Subscribers ----

    def _infer_msg_type_from_callback(self, callback: Callable) -> Any:
        sig = inspect.signature(callback)
        params = list(sig.parameters.values())
        if not params:
            raise TypeError("Callback must accept at least one argument for the message")
        ann = params[0].annotation
        if ann is inspect._empty:
            raise TypeError(
                "msg_type not provided and cannot be inferred from callback "
                "annotation. Either pass msg_type explicitly or annotate the "
                "first parameter of the callback with the message type."
            )
        return ann

    def sub(
        self,
        topic: str,
        msg_type: Optional[Any] = None,
        callback: Optional[Callable] = None,
        *,
        qos: Optional[Any] = None,
        queue_size: int = 10,
        name: Optional[str] = None,  # reserved
    ) -> LxSubscription:
        if callback is None:
            raise ValueError("callback must be provided")

        if msg_type is None:
            msg_type = self._infer_msg_type_from_callback(callback)

        qos_profile = _make_qos(qos, queue_size)

        # We could wrap callback if we want to inject 'self' or metadata later.
        subscription = self._node.create_subscription(
            msg_type,
            topic,
            callback,
            qos_profile=qos_profile,
        )
        wrapper = LxSubscription(self, topic, subscription)
        self._subscriptions.append(wrapper)
        return wrapper

    # ---- Timers ----

    def timer(self, period_sec: float, callback: Callable[[], Any]) -> Any:
        timer = self._node.create_timer(period_sec, callback)
        self._timers.append(timer)
        return timer

    # ---- Services ----

    def service(
        self,
        name: str,
        srv_type: Any,
        handler: Callable[[Any], Any],
    ) -> LxService:
        """
        Create a ROS1-style service server.

        handler: request -> response
        - request is srv_type.Request
        - response is srv_type.Response (or compatible object)
        """

        def internal_callback(request, response):
            user_resp = handler(request)

            if isinstance(user_resp, srv_type.Response):
                # Copy fields into provided response instance
                for field_name in user_resp.get_fields_and_field_types().keys():
                    setattr(response, field_name, getattr(user_resp, field_name))
            elif isinstance(user_resp, dict):
                for k, v in user_resp.items():
                    setattr(response, k, v)
            else:
                # Fallback: assume user_resp is already response and just return it
                response = user_resp

            return response

        service = self._node.create_service(srv_type, name, internal_callback)
        wrapper = LxService(self, name, service)
        self._services.append(wrapper)
        return wrapper

    def service_client(
        self,
        name: str,
        srv_type: Any,
    ) -> LxServiceClient:
        client = self._node.create_client(srv_type, name)
        return LxServiceClient(self, name, srv_type, client)

    # ---- Parameters ----

    def declare_param(self, name: str, default: Any = None, descriptor: Any = None) -> Any:
        return self._node.declare_parameter(name, default_value=default, descriptor=descriptor)

    def get_param(self, name: str, default: Any = None, type: Optional[type] = None) -> Any:
        if not self._node.has_parameter(name):
            value = default
        else:
            value = self._node.get_parameter(name).value

        if type is not None and value is not None:
            try:
                value = type(value)
            except Exception:
                raise TypeError(f"Cannot cast parameter '{name}' value '{value}' to {type}")

        return value
