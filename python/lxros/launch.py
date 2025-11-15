# lxros/launch.py

from __future__ import annotations

from typing import Any, Dict, Mapping, MutableMapping, Optional, Sequence, Iterable, Union

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, ThisLaunchFileDir

from launch_ros.actions import Node as RosNode, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


ParamLike = Union[Mapping[str, Any], Any]  # dict or Substitution/path-like


def _pkg_path(pkg: str, relpath: str) -> PathJoinSubstitution:
    """Return a path inside a package share directory."""
    return PathJoinSubstitution([FindPackageShare(pkg), relpath])


def _here_path(relpath: str) -> PathJoinSubstitution:
    """Return a path relative to the current launch file."""
    return PathJoinSubstitution([ThisLaunchFileDir(), relpath])


def launch(*actions: Action, description: Optional[str] = None) -> LaunchDescription:
    """
    Create a LaunchDescription from a list of actions.

    Example:
        return launch(
            node(...),
            group("robot1", node(...)),
        )
    """
    ld = LaunchDescription()
    if description is not None:
        ld.description = description
    for act in actions:
        ld.add_action(act)
    return ld


def node(
    name: Optional[str],
    package: str,
    executable: str,
    *,
    namespace: Optional[str] = None,
    remap: Optional[Union[Mapping[str, str], Sequence[tuple[str, str]]]] = None,
    params: Optional[Union[ParamLike, Sequence[ParamLike]]] = None,
    args: Optional[Union[str, Sequence[str]]] = None,
    output: str = "screen",
    respawn: bool = False,
    env: Optional[Mapping[str, str]] = None,
    **kwargs: Any,
) -> RosNode:
    """
    Convenience wrapper for launch_ros.actions.Node.

    - remap: dict {'from': 'to'} or list of (from, to)
    - params: dict, YAML path/substitution, or list mixing both
    - args: string or list of strings (extra CLI arguments)
    """

    # Normalize remappings
    remappings: Optional[Sequence[tuple[str, str]]] = None
    if remap:
        if isinstance(remap, Mapping):
            remappings = [(str(k), str(v)) for k, v in remap.items()]
        else:
            remappings = [(str(a), str(b)) for a, b in remap]

    # Normalize parameters
    parameters: Optional[Sequence[ParamLike]] = None
    if params is not None:
        if isinstance(params, (list, tuple)):
            parameters = list(params)
        else:
            parameters = [params]

    # Normalize arguments
    arguments: Optional[Sequence[str]] = None
    if args is not None:
        if isinstance(args, str):
            arguments = [args]
        else:
            arguments = list(args)

    return RosNode(
        package=package,
        executable=executable,
        name=name,
        namespace=namespace,
        remappings=remappings,
        parameters=parameters,
        arguments=arguments,
        output=output,
        respawn=respawn,
        env=env,
        **kwargs,
    )


def group(namespace: Optional[str], *actions: Action) -> GroupAction:
    """
    Group actions, optionally pushing a ROS namespace.

    Example:
        group("robot1",
            node(...),
            node(...),
        )
    """
    children: list[Action] = []
    if namespace:
        children.append(PushRosNamespace(namespace))
    children.extend(actions)
    return GroupAction(actions=children)


def include(
    pkg: str,
    launch_file: str,
    *,
    args: Optional[Mapping[str, Any]] = None,
) -> IncludeLaunchDescription:
    """
    Include another launch file from a package.

    Example:
        include("franka_fr3_moveit_config", "moveit.launch.py",
                args={"robot_ip": "dont-care", "use_fake_hardware": "true"})
    """
    launch_path = _pkg_path(pkg, launch_file)
    source = PythonLaunchDescriptionSource(launch_path)

    launch_arguments = None
    if args:
        # args is a mapping name -> value (converted to str)
        launch_arguments = [(str(k), str(v)) for k, v in args.items()]

    return IncludeLaunchDescription(source, launch_arguments=launch_arguments)


def params(source: Union[Mapping[str, Any], str], *, pkg: Optional[str] = None) -> ParamLike:
    """
    Normalize parameter source.

    - If source is a dict, return it as-is.
    - If source is a string, interpret as YAML path:
        - if pkg is given, resolve inside that package share
        - else, resolve relative to this launch file
    """
    if isinstance(source, Mapping):
        return source
    if not isinstance(source, str):
        raise TypeError(f"params() expected dict or str, got {type(source)}")

    if pkg:
        return _pkg_path(pkg, source)
    return _here_path(source)


def robot_description(
    xacro_path: str,
    *,
    pkg: Optional[str] = None,
    mappings: Optional[Mapping[str, Any]] = None,
    param: str = "robot_description",
) -> Dict[str, Any]:
    """
    Build a 'robot_description' parameter from a xacro file.

    Example:
        params=robot_description("urdf/robot.urdf.xacro", pkg="my_robot_description")
    """
    if pkg:
        xacro_full = _pkg_path(pkg, xacro_path)
    else:
        xacro_full = _here_path(xacro_path)

    cmd: list[Any] = ["xacro", xacro_full]
    if mappings:
        for k, v in mappings.items():
            cmd.append(f"{k}:={v}")

    return {param: Command(cmd)}


def rviz(
    config: Optional[str] = None,
    *,
    pkg: Optional[str] = None,
    name: Optional[str] = None,
    namespace: Optional[str] = None,
) -> RosNode:
    """
    Convenience wrapper for launching RViz2.

    Example:
        rviz("config/robot_view.rviz", pkg="my_robot_bringup")
    """
    args: Optional[Sequence[str]] = None
    if config is not None:
        if pkg:
            config_path = _pkg_path(pkg, config)
        else:
            config_path = _here_path(config)
        args = ["-d", config_path]

    return node(
        name=name or "rviz",
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        args=args,
    )


__all__ = [
    "launch",
    "node",
    "group",
    "include",
    "params",
    "robot_description",
    "rviz",
]
