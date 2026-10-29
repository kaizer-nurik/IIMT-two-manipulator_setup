"""Two direct drivers; no legacy ros2_control hardware or controller manager."""

import ipaddress
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_defaults():
    share = Path(get_package_share_directory("iimt_dual_control"))
    result = {
        "use_mock_hardware": "true",
        "enable_motion": "false",
        "robot_type": "5",
        "dispatch_lead_time_s": "1.0",
        "dispatch_margin_s": "0.2",
    }
    for arm in ("left", "right"):
        result.update(
            {
                f"{arm}_robot_ip": "",
                f"{arm}_parameters_file": str(share / f"config/{arm}_driver.yaml"),
                f"{arm}_command_rate": "125.0",
                f"{arm}_read_rate": "125.0",
                f"{arm}_trajectory_advance_ms": "0.0",
                f"{arm}_feedback_transport": "tcp",
                f"{arm}_read_timeout": "0.1",
            }
        )
    return result


def load_profile(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or len(data) != 1:
        raise ValueError(
            "Arm profile must contain exactly one ROS node parameter mapping"
        )
    return next(iter(data.values()))["ros__parameters"]


def launch_setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)

    def boolean(name):
        text = value(name).lower()
        if text not in ("true", "false"):
            raise ValueError(f"{name} must be true or false")
        return text == "true"

    mock, motion = boolean("use_mock_hardware"), boolean("enable_motion")
    if value("robot_type") != "5":
        raise ValueError("Dual description and limits support robot_type=5 only")
    addresses = [value(f"{arm}_robot_ip") for arm in ("left", "right")]
    if not mock:
        parsed = [ipaddress.IPv4Address(address) for address in addresses]
        if len(set(parsed)) != 2 or any(
            ip.is_unspecified or ip.is_multicast for ip in parsed
        ):
            raise ValueError(
                "Real mode requires two distinct valid robot IPv4 addresses"
            )
    nodes = []
    coordinator = {
        key: float(value(key)) for key in ("dispatch_lead_time_s", "dispatch_margin_s")
    }
    for arm, address in zip(("left", "right"), addresses):
        parameters = load_profile(value(f"{arm}_parameters_file"))
        parameters.update(
            use_mock_hardware=mock,
            enable_motion=motion,
            joint_prefix=f"{arm}_",
            robot_ip=address or "127.0.0.1",
        )
        for key in (
            "command_rate",
            "read_rate",
            "trajectory_advance_ms",
            "read_timeout",
        ):
            parameters[key] = float(value(f"{arm}_{key}"))
        parameters["feedback_transport"] = value(f"{arm}_feedback_transport")
        coordinator[f"{arm}_trajectory_advance_ms"] = parameters[
            "trajectory_advance_ms"
        ]
        nodes.append(
            Node(
                package="iimt_ros2_driver",
                executable="iimt_ros2_driver",
                name="iimt_ros2_driver",
                namespace=arm,
                parameters=[parameters],
                output="screen",
                on_exit=Shutdown(reason=f"{arm} driver exited"),
            )
        )
    required = (
        coordinator["dispatch_margin_s"]
        + max(coordinator[f"{arm}_trajectory_advance_ms"] for arm in ("left", "right"))
        / 1000
    )
    if coordinator["dispatch_lead_time_s"] < required:
        raise ValueError(
            "Dispatch lead time must cover the largest arm advance and margin"
        )
    nodes.append(
        Node(
            package="iimt_dual_control",
            executable="dual_coordinator",
            parameters=[coordinator],
            output="screen",
            on_exit=Shutdown(reason="Dual coordinator exited"),
        )
    )
    return nodes
