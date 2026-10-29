"""CI-05 MoveIt with the direct SDK driver, without ros2_control or FIFO bridge."""
from pathlib import Path
import math

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def launch_setup(context):
    config = Path(get_package_share_directory("iimt_ros2_moveit_config"))
    description = Path(get_package_share_directory("iimt_ros2_description"))
    driver = Path(get_package_share_directory("iimt_ros2_driver"))
    if LaunchConfiguration("robot_type").perform(context) != "5":
        raise ValueError("The direct driver currently supports CI-05 (robot_type=5) only")
    rviz = LaunchConfiguration("launch_rviz").perform(context).lower()
    if rviz not in ("true", "false"):
        raise ValueError("launch_rviz must be true or false")
    robot = {"robot_description": (description / "urdf/iimt5.urdf").read_text()}
    semantic = {"robot_description_semantic": (config / "srdf/iimt5.srdf").read_text()}
    kinematics = load_yaml(config / "config/kinematics.yaml")
    driver_config = load_yaml(Path(LaunchConfiguration("parameters_file").perform(context)))
    driver_parameters = driver_config["/iimt_ros2_driver"]["ros__parameters"]
    bounds = driver_parameters["limits"]
    advance_ms = float(LaunchConfiguration("trajectory_advance_ms").perform(context)
                       or driver_parameters.get("trajectory_advance_ms", 0.0))
    if not math.isfinite(advance_ms) or not 0 <= advance_ms <= 1000:
        raise ValueError("trajectory_advance_ms must be finite and within 0..1000 ms")
    advance = advance_ms / 1000.0
    execution = load_yaml(config / "config/trajectory_execution.yaml")["move_group"]["ros__parameters"]
    # Immediate goals include the advance pre-roll before their nominal start.
    execution["trajectory_execution"]["allowed_goal_duration_margin"] += advance
    rate = float(LaunchConfiguration("command_rate").perform(context)
                 or driver_config["/iimt_ros2_driver"]["ros__parameters"]["command_rate"])
    if not math.isfinite(rate) or not 1 <= rate <= 125:
        raise ValueError("command_rate must be finite and within 1..125 Hz")
    planning_scale = float(LaunchConfiguration("planning_limit_scale").perform(context))
    if not math.isfinite(planning_scale) or not 0 < planning_scale <= 1:
        raise ValueError("planning_limit_scale must be finite and within (0, 1]")
    names = ["shoulder_joint", "upper_arm_joint", "forearm_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    for key in ("min_position", "max_position", "max_velocity", "max_acceleration", "max_step"):
        if len(bounds[key]) != 6 or not all(math.isfinite(v) for v in bounds[key]):
            raise ValueError(f"limits.{key} must contain six finite values")
    planning = load_yaml(config / "config/joint_limits.yaml")
    for i, name in enumerate(names):
        if (bounds["min_position"][i] >= bounds["max_position"][i]
                or any(bounds[key][i] <= 0 for key in ("max_velocity", "max_acceleration", "max_step"))):
            raise ValueError(f"Invalid driver limits for {name}")
        planning["joint_limits"][name].update({
            "has_position_limits": True,
            "min_position": bounds["min_position"][i],
            "max_position": bounds["max_position"][i],
            "has_velocity_limits": True,
            # Respect the driver's nominal per-tick step as well as velocity limit.
            "max_velocity": planning_scale * min(bounds["max_velocity"][i], bounds["max_step"][i] * rate),
            "has_acceleration_limits": True,
            "max_acceleration": planning_scale * bounds["max_acceleration"][i],
        })
    limits = {"robot_description_planning": planning}
    pipeline = {"move_group": {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": " ".join([
            "default_planner_request_adapters/AddTimeOptimalParameterization",
            "default_planner_request_adapters/ResolveConstraintFrames",
            "default_planner_request_adapters/FixWorkspaceBounds",
            "default_planner_request_adapters/FixStartStateBounds",
            "default_planner_request_adapters/FixStartStateCollision",
            "default_planner_request_adapters/FixStartStatePathConstraints",
        ]),
        "start_state_max_bounds_error": 0.02,
        **load_yaml(config / "config/ompl_planning.yaml"),
    }}
    controllers = {
        "moveit_simple_controller_manager": load_yaml(config / "config/moveit_controllers.yaml"),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    actions = [
        LogInfo(msg=f"MoveIt planning limits use {planning_scale:.3f} of driver velocity/acceleration "
                    "limits; interpolation may overshoot waypoint derivatives. Driver hard limits unchanged."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver / "launch/iimt_ros2_driver.launch.py")),
            launch_arguments={name: LaunchConfiguration(name) for name in (
                "robot_ip", "enable_motion", "use_mock_hardware", "command_rate", "parameters_file",
                "feedback_transport", "read_timeout", "read_rate", "trajectory_advance_ms"
            )}.items(),
        ),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[robot], output="screen"),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[robot, semantic, kinematics, limits, pipeline, controllers,
                         execution]),
    ]
    if rviz == "true":
        actions.append(Node(
            package="rviz2", executable="rviz2", name="rviz2_moveit", output="screen",
            arguments=["-d", str(config / "rviz/moveit.rviz")],
            parameters=[robot, semantic, kinematics, limits, pipeline],
        ))
    return actions


def generate_launch_description():
    driver = Path(get_package_share_directory("iimt_ros2_driver"))
    defaults = {
        "robot_ip": "", "robot_type": "5",
        "enable_motion": "false", "use_mock_hardware": "false",
        "command_rate": "", "launch_rviz": "true",
        "feedback_transport": "", "read_timeout": "", "read_rate": "",
        "trajectory_advance_ms": "",
        "planning_limit_scale": "0.5",
        "parameters_file": str(driver / "config/driver.yaml"),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_setup),
    ])
