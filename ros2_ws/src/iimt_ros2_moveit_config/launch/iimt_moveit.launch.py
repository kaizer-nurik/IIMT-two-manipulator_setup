"""IIMT-CI-05: MoveIt -> ros2_control/JTC -> measured IIMT hardware.

Never starts the legacy trajectory/FIFO driver alongside the hardware plugin.
"""
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def load_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def boolean(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def launch_setup(context):
    config = Path(get_package_share_directory("iimt_ros2_moveit_config"))
    description = Path(get_package_share_directory("iimt_ros2_description"))
    robot_type = LaunchConfiguration("robot_type").perform(context)
    mock = boolean(context, "use_mock_hardware")
    motion = boolean(context, "enable_motion")
    sim_time = boolean(context, "use_sim_time")
    if robot_type not in ("3", "5", "10") or (not mock and robot_type != "5"):
        raise ValueError("Real hardware supports verified CI-05 limits (robot_type=5); other models are mock-only")
    if sim_time and not mock:
        raise ValueError("Real hardware requires use_sim_time=false")
    hardware_file = Path(LaunchConfiguration("hardware_parameters_file").perform(context)).resolve()
    controllers_file = Path(LaunchConfiguration("controllers_file").perform(context)).resolve()
    hardware_yaml = load_yaml(hardware_file)
    hardware = hardware_yaml.get("/iimt_hardware", hardware_yaml.get("iimt_hardware", {})).get("ros__parameters", {})
    controllers = load_yaml(controllers_file)
    update_rate = controllers["controller_manager"]["ros__parameters"]["update_rate"]
    servo_period = hardware["servo"]["period"]
    if update_rate <= 0 or abs(update_rate * servo_period - 1.0) > 1e-6:
        raise ValueError("controller_manager.update_rate must equal 1 / servo.period")
    sdk_path = LaunchConfiguration("sdk_library_path").perform(context)
    if not sdk_path and not mock:
        sdk_path = str(Path(get_package_prefix("iimt_ros2_driver")) / "lib/iimt_ros2_driver/libiimtcl.so")
    robot_xml = xacro.process_file(
        str(description / "urdf/iimt_hardware.urdf.xacro"), mappings={
            "robot_type": robot_type,
            "robot_ip": LaunchConfiguration("robot_ip").perform(context),
            "hardware_parameters_file": str(hardware_file),
            "sdk_library_path": sdk_path,
            "controller_update_rate": str(update_rate),
            "enable_motion": str(motion).lower(),
            "use_mock_hardware": str(mock).lower(),
        })
    # The common geometry wrapper must match the selected model's SRDF robot name.
    robot_xml.documentElement.setAttribute("name", f"iimt{robot_type}")
    robot_description = {"robot_description": robot_xml.toxml()}
    semantic = {"robot_description_semantic": (config / f"srdf/iimt{robot_type}.srdf").read_text()}
    kinematics = load_yaml(config / "config/kinematics.yaml")
    limits = {"robot_description_planning": load_yaml(config / "config/joint_limits.yaml")}
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
    # Execution parameters are ROS node parameters in a separate overrideable file.
    execution_file = LaunchConfiguration("moveit_parameters_file").perform(context)
    moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml(config / "config/moveit_controllers.yaml"),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
                           parameters=[robot_description, {"use_sim_time": sim_time}], output="screen")
    manager = Node(package="controller_manager", executable="ros2_control_node", output="screen",
                   parameters=[str(controllers_file), {"use_sim_time": sim_time}],
                   remappings=[("~/robot_description", "/robot_description")])
    broadcaster = Node(package="controller_manager", executable="spawner", output="screen",
                       arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                                  "--controller-manager-timeout", "30"])
    # Read-only commissioning must not claim command interfaces.
    trajectory_arguments = ["iimt_group_controller", "--controller-manager", "/controller_manager",
                            "--controller-manager-timeout", "30"]
    if not mock and not motion:
        trajectory_arguments.append("--inactive")
    trajectory = Node(package="controller_manager", executable="spawner", output="screen",
                      arguments=trajectory_arguments)
    move_group = Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                      parameters=[robot_description, semantic, kinematics, pipeline, limits,
                                  moveit_controllers, execution_file, {"use_sim_time": sim_time}])

    def start_trajectory(event, _context):
        if event.returncode != 0:
            return [Shutdown(reason="joint_state_broadcaster failed to start")]
        return [trajectory]

    def start_moveit(event, _context):
        if event.returncode != 0:
            return [Shutdown(reason="iimt_group_controller failed to load/configure")]
        actions = [move_group]
        if boolean(context, "launch_rviz"):
            actions.append(Node(package="rviz2", executable="rviz2", name="rviz2_moveit", output="log",
                                arguments=["-d", str(config / "rviz/moveit.rviz")],
                                parameters=[robot_description, semantic, kinematics, pipeline, limits,
                                            {"use_sim_time": sim_time}]))
        if boolean(context, "db"):
            actions.append(Node(package="warehouse_ros_mongo", executable="mongo_wrapper_ros.py",
                                parameters=[{"warehouse_port": 33829, "warehouse_host": "localhost",
                                             "warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"}]))
        return actions

    return [
        RegisterEventHandler(OnProcessExit(target_action=manager,
                             on_exit=[Shutdown(reason="controller_manager exited; execution unavailable")])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=start_trajectory)),
        RegisterEventHandler(OnProcessExit(target_action=trajectory, on_exit=start_moveit)),
        state_publisher, manager, broadcaster,
    ]


def generate_launch_description():
    config = Path(get_package_share_directory("iimt_ros2_moveit_config"))
    control = Path(get_package_share_directory("iimt_ros2_control"))
    defaults = {
        "robot_ip": ("192.168.50.148", "Robot controller address"),
        "robot_type": ("5", "Real CI-05, or 3/10 in explicit mock mode"),
        "enable_motion": ("false", "Explicitly allow real position commands; no automatic power/brake release"),
        "use_mock_hardware": ("false", "Explicit simulation: never use for physical accuracy measurements"),
        "use_sim_time": ("false", "Simulation clock is forbidden for real hardware"),
        "launch_rviz": ("true", "Start RViz"),
        "db": ("false", "Start optional warehouse database"),
        "hardware_parameters_file": (str(control / "config/hardware.yaml"), "Startup-only /iimt_hardware node parameters"),
        "controllers_file": (str(config / "config/ros2_controllers.yaml"), "Controller manager and JTC node parameters"),
        "moveit_parameters_file": (str(config / "config/trajectory_execution.yaml"), "MoveIt execution/monitor parameters"),
        "sdk_library_path": ("", "Optional absolute path to the original IIMT SDK .so"),
    }
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value, description=description)
                              for name, (value, description) in defaults.items()] + [OpaqueFunction(function=launch_setup)])
