"""Dual CI-05 geometry, MoveIt and synchronized direct SDK drivers."""

import math
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from iimt_dual_control.bringup import launch_defaults, load_profile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

JOINTS = (
    "shoulder_joint",
    "upper_arm_joint",
    "forearm_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def load_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def finite_float(text, name, lower, upper):
    value = float(text)
    if not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"{name} must be finite and within {lower}..{upper}")
    return value


def base_transforms(context):
    transforms = {}
    for name in ("left_base_xyz", "left_base_rpy", "right_base_xyz", "right_base_rpy"):
        text = LaunchConfiguration(name).perform(context)
        values = [float(value) for value in text.split()]
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise ValueError(
                f"{name} must contain three finite space-separated numbers"
            )
        transforms[name] = text
    return transforms


def planning_limits(context):
    scale = finite_float(
        LaunchConfiguration("planning_limit_scale").perform(context),
        "planning_limit_scale",
        0.000001,
        1.0,
    )
    limits = {}
    for side in ("left", "right"):
        profile = load_profile(
            LaunchConfiguration(f"{side}_parameters_file").perform(context)
        )
        bounds = profile["limits"]
        rate = finite_float(
            LaunchConfiguration(f"{side}_command_rate").perform(context),
            f"{side}_command_rate",
            1.0,
            125.0,
        )
        for key in (
            "min_position",
            "max_position",
            "max_velocity",
            "max_acceleration",
            "max_step",
        ):
            if len(bounds[key]) != 6 or not all(math.isfinite(v) for v in bounds[key]):
                raise ValueError(f"{side}: limits.{key} requires six finite values")
        for index, name in enumerate(JOINTS):
            if bounds["min_position"][index] >= bounds["max_position"][index] or any(
                bounds[key][index] <= 0
                for key in ("max_velocity", "max_acceleration", "max_step")
            ):
                raise ValueError(f"{side}: invalid limits for {name}")
            limits[f"{side}_{name}"] = {
                "has_position_limits": True,
                "min_position": bounds["min_position"][index],
                "max_position": bounds["max_position"][index],
                "has_velocity_limits": True,
                "max_velocity": scale
                * min(bounds["max_velocity"][index], bounds["max_step"][index] * rate),
                "has_acceleration_limits": True,
                "max_acceleration": scale * bounds["max_acceleration"][index],
            }
    return {"robot_description_planning": {"joint_limits": limits}}


def launch_setup(context):
    config = Path(get_package_share_directory("iimt_dual_moveit_config"))
    description = Path(get_package_share_directory("iimt_dual_description"))
    control = Path(get_package_share_directory("iimt_dual_control"))
    rviz = LaunchConfiguration("launch_rviz").perform(context).lower()
    if rviz not in ("true", "false"):
        raise ValueError("launch_rviz must be true or false")
    transforms = base_transforms(context)
    workspace_margins = {
        name: finite_float(
            LaunchConfiguration(name).perform(context), name, 0.000001, 100.0
        )
        for name in (
            "workspace_margin",
            "workspace_x_min_margin",
            "workspace_y_min_margin",
            "workspace_x_max_margin",
            "workspace_y_max_margin",
        )
    }
    floor_text = LaunchConfiguration("floor_z").perform(context)
    floor_parameters = (
        {}
        if floor_text == "auto"
        else {"floor_z": finite_float(floor_text, "floor_z", -math.inf, math.inf)}
    )
    robot = {
        "robot_description": xacro.process_file(
            str(description / "urdf/iimt_dual.urdf.xacro"),
            mappings=transforms,
        ).toxml()
    }
    semantic = {
        "robot_description_semantic": (config / "srdf/iimt_dual.srdf").read_text()
    }
    kinematics = load_yaml(config / "config/kinematics.yaml")
    limits = planning_limits(context)
    adapters = (
        "AddTimeOptimalParameterization",
        "ResolveConstraintFrames",
        "FixWorkspaceBounds",
        "FixStartStateBounds",
        "FixStartStateCollision",
        "FixStartStatePathConstraints",
    )
    pipeline = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": " ".join(
                f"default_planner_request_adapters/{a}" for a in adapters
            ),
            "start_state_max_bounds_error": 0.02,
            **load_yaml(config / "config/ompl_planning.yaml"),
        }
    }
    controllers = {
        "moveit_simple_controller_manager": load_yaml(
            config / "config/moveit_controllers.yaml"
        ),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    execution = load_yaml(config / "config/trajectory_execution.yaml")
    lead = finite_float(
        LaunchConfiguration("dispatch_lead_time_s").perform(context),
        "dispatch_lead_time_s",
        0.0,
        60.0,
    )
    advance = (
        max(
            finite_float(
                LaunchConfiguration(f"{s}_trajectory_advance_ms").perform(context),
                f"{s}_trajectory_advance_ms",
                0.0,
                1000.0,
            )
            for s in ("left", "right")
        )
        / 1000.0
    )
    execution["trajectory_execution"]["allowed_goal_duration_margin"] += lead + advance
    actions = [
        LogInfo(
            msg="Dual bases use configured transforms (defaults are illustrative, not calibration). "
            "All groups execute through the synchronized dual coordinator."
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(control / "launch/dual_control.launch.py")
            ),
            launch_arguments={
                name: LaunchConfiguration(name) for name in launch_defaults()
            }.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot,
                semantic,
                kinematics,
                limits,
                pipeline,
                controllers,
                execution,
            ],
        ),
    ]
    actions.append(
        Node(
            package="iimt_dual_moveit_config",
            executable="workspace_scene.py",
            name="iimt_workspace",
            output="screen",
            on_exit=Shutdown(reason="Workspace scene node exited"),
            parameters=[
                {
                    **{
                        name: [float(v) for v in transforms[name].split()]
                        for name in ("left_base_xyz", "left_base_rpy", "right_base_xyz")
                    },
                    **workspace_margins,
                    **floor_parameters,
                }
            ],
        )
    )
    if rviz == "true":
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="dual_rviz",
                output="screen",
                arguments=["-d", str(config / "rviz/dual.rviz")],
                parameters=[robot, semantic, kinematics, limits, pipeline],
            )
        )
    return actions


def generate_launch_description():
    defaults = {
        **launch_defaults(),
        "launch_rviz": "true",
        "planning_limit_scale": "0.5",
        "workspace_margin": "0.5",
        "floor_z": "auto",
        **{
            f"workspace_{side}_margin": LaunchConfiguration("workspace_margin")
            for side in ("x_min", "y_min", "x_max", "y_max")
        },
        "left_base_xyz": "-0.0008297 -0.5380000 0.0818739",
        "left_base_rpy": "0 0 -0.6981317008",
        "right_base_xyz": "-0.345440489323 0.411679944176 -0.082000000000",
        "right_base_rpy": "0 0 0",
    }
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in defaults.items()
            ],
            OpaqueFunction(function=launch_setup),
        ]
    )
