"""Single-arm direct FollowJointTrajectory driver; no legacy queue bridge."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    # Профиль измерений имеет приоритет над пустыми CLI-аргументами.
    overrides = {
        name: ParameterValue(LaunchConfiguration(name), value_type=bool)
        for name in ("enable_motion", "use_mock_hardware")
    }
    for name, kind in (("robot_ip", str), ("joint_prefix", str), ("command_rate", float),
                       ("trajectory_advance_ms", float),
                       ("feedback_transport", str), ("read_timeout", float), ("read_rate", float)):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = kind(value)
    return [Node(
        package="iimt_ros2_driver", executable="iimt_ros2_driver",
        name="iimt_ros2_driver", output="screen",
        parameters=[LaunchConfiguration("parameters_file"), overrides],
        on_exit=Shutdown(reason="Direct IIMT driver exited"),
    )]


def generate_launch_description():
    share = Path(get_package_share_directory("iimt_ros2_driver"))
    defaults = {
        "robot_ip": "",
        "joint_prefix": "",
        "enable_motion": "false",
        "use_mock_hardware": "false",
        "command_rate": "",
        "trajectory_advance_ms": "",
        "feedback_transport": "",
        "read_timeout": "",
        "read_rate": "",
        "parameters_file": str(share / "config/driver.yaml"),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_setup),
    ])
