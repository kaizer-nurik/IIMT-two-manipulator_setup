#!/usr/bin/env python3
"""Offline single/dual execution and RRTConnect acceptance test (fake hardware only)."""

import csv
import json
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = os.environ.get("IIMT_TEST_DOMAIN_ID", "216")
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetStateValidity
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parents[1]
JOINTS = [
    "shoulder_joint",
    "upper_arm_joint",
    "forearm_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def wait(node, future, timeout=10):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError(f"ROS request did not finish within {timeout}s")
    return future.result()


def call(node, service_type, name, request):
    client = node.create_client(service_type, name)
    try:
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError(f"Missing service: {name}")
        return wait(node, client.call_async(request))
    finally:
        node.destroy_client(client)


def spin_until(node, predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise RuntimeError("Condition timed out")
        rclpy.spin_once(node, timeout_sec=0.05)


def execute(node, action_type, name, goal, timeout):
    client = ActionClient(node, action_type, name)
    try:
        if not client.wait_for_server(timeout_sec=20):
            raise RuntimeError(f"Missing action: {name}")
        started = time.monotonic()
        handle = wait(node, client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError(f"Goal rejected by {name}")
        result = wait(node, handle.get_result_async(), timeout)
        code = result.result.error_code
        expected = 0 if action_type is FollowJointTrajectory else 1
        actual = code if isinstance(code, int) else code.val
        if result.status != 4 or actual != expected:
            raise RuntimeError(
                f"{name}: status={result.status}, result={result.result}"
            )
        return time.monotonic() - started
    finally:
        client.destroy()


def goal(node, names, start, target, duration):
    result = FollowJointTrajectory.Goal()
    result.trajectory.joint_names = names
    result.trajectory.header.stamp = (
        node.get_clock().now() + Duration(seconds=1.5)
    ).to_msg()
    for seconds, positions in ((0.0, start), (duration, target)):
        point = JointTrajectoryPoint(
            positions=positions,
            velocities=[0.0] * len(names),
            accelerations=[0.0] * len(names),
        )
        point.time_from_start = Duration(seconds=seconds).to_msg()
        result.trajectory.points.append(point)
    return result


def csv_goal(node, names, rows, csv_arms):
    """Use only original CSV positions and timestamps; do not synthesize derivatives."""
    result = FollowJointTrajectory.Goal()
    result.trajectory.joint_names = names
    result.trajectory.header.stamp = (
        node.get_clock().now() + Duration(seconds=3.0)
    ).to_msg()
    for row in rows:
        point = JointTrajectoryPoint(
            positions=[
                float(row[f"{arm}_q{axis}_rad"])
                for arm in csv_arms
                for axis in range(1, 7)
            ]
        )
        point.time_from_start = Duration(seconds=float(row["t"])).to_msg()
        result.trajectory.points.append(point)
    assert len(result.trajectory.points) == len(rows)
    assert all(
        not point.velocities and not point.accelerations and not point.effort
        for point in result.trajectory.points
    )
    return result


def state(names, positions):
    result = RobotState()
    result.joint_state = JointState(name=names, position=positions)
    result.is_diff = False
    return result


def validity(node, names, positions):
    return call(
        node,
        GetStateValidity,
        "/check_state_validity",
        GetStateValidity.Request(
            robot_state=state(names, positions), group_name="both_arms"
        ),
    )


def check(mode):
    dual = mode == "dual"
    arms = ["left", "right"] if dual else [""]
    names = [f"{arm}_{joint}" if arm else joint for arm in arms for joint in JOINTS]
    package, launch = (
        ("iimt_dual_moveit_config", "dual_moveit.launch.py")
        if dual
        else ("iimt_ros2_moveit_config", "iimt_driver_moveit.launch.py")
    )
    action = (
        "/both_arms_controller/follow_joint_trajectory"
        if dual
        else "/iimt_group_controller/follow_joint_trajectory"
    )
    command = [
        "ros2",
        "launch",
        package,
        launch,
        "use_mock_hardware:=true",
        "enable_motion:=true",
        "launch_rviz:=false",
    ]
    command += (
        ["left_robot_ip:=127.0.0.1", "right_robot_ip:=127.0.0.2"]
        if dual
        else ["robot_ip:=127.0.0.1"]
    )
    if not dual:
        command += ["command_rate:=125.0", "read_rate:=125.0"]
    with (ROOT / "data/trajectory_both_TimeGrid_30Hz.csv").open() as csv_file:
        rows = list(csv.DictReader(csv_file))
    csv_arms = ["left", "right"] if dual else ["left"]
    seed = [
        float(rows[0][f"{arm}_q{axis}_rad"]) for arm in csv_arms for axis in range(1, 7)
    ]
    report = {"mode": mode, "mock_only": True, "domain": os.environ["ROS_DOMAIN_ID"]}
    with tempfile.TemporaryDirectory(prefix=f"iimt-{mode}-execution-") as temporary:
        log_path = Path(temporary) / "launch.log"
        with log_path.open("w") as log:
            process = subprocess.Popen(
                command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
            rclpy.init()
            node = rclpy.create_node("iimt_mock_execution_verifier")
            samples, timing = {}, {}
            subscriptions = []
            try:
                for arm in arms:
                    prefix = f"/{arm}" if arm else ""
                    driver = prefix + "/iimt_ros2_driver"
                    response = call(
                        node,
                        GetParameters,
                        driver + "/get_parameters",
                        GetParameters.Request(
                            names=[
                                "use_mock_hardware",
                                "enable_motion",
                                "robot_ip",
                                "command_rate",
                                "read_rate",
                            ]
                        ),
                    )
                    assert (
                        response.values[0].bool_value and response.values[1].bool_value
                    ), "Refusing goals: not enabled mock hardware"
                    assert response.values[2].string_value.startswith(
                        "127."
                    ), "Refusing non-loopback robot IP"
                    assert response.values[3].double_value == 125.0
                    assert response.values[4].double_value == 125.0
                    report["command_rate_hz"] = report["read_rate_hz"] = 125.0
                    subscriptions.append(
                        node.create_subscription(
                            JointState,
                            prefix + "/joint_states",
                            lambda message, arm=arm: samples.update({arm: message}),
                            qos_profile_sensor_data,
                        )
                    )
                    qos = QoSProfile(
                        depth=10,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    )
                    subscriptions.append(
                        node.create_subscription(
                            DiagnosticArray,
                            driver + "/trajectory_timing",
                            lambda message, arm=arm: timing.update(
                                {
                                    arm: {
                                        entry.key: entry.value
                                        for status in message.status
                                        for entry in status.values
                                    }
                                }
                            ),
                            qos,
                        )
                    )
                spin_until(node, lambda: len(samples) == len(arms))
                measured = lambda: [
                    dict(zip(samples[arm].name, samples[arm].position))[name]
                    for arm in arms
                    for name in names
                    if (name.startswith(arm + "_") if arm else True)
                ]
                if dual:
                    objects = set()
                    deadline = time.monotonic() + 20
                    while "iimt_workspace_floor" not in objects:
                        scene = call(
                            node,
                            GetPlanningScene,
                            "/get_planning_scene",
                            GetPlanningScene.Request(
                                components=PlanningSceneComponents(components=16)
                            ),
                        ).scene
                        objects = {item.id for item in scene.world.collision_objects}
                        if time.monotonic() > deadline:
                            raise RuntimeError("Workspace/floor was not installed")
                    assert objects >= {
                        "iimt_workspace_floor",
                        "iimt_workspace_x_min",
                        "iimt_workspace_x_max",
                        "iimt_workspace_y_min",
                        "iimt_workspace_y_max",
                    }
                    report["collision_objects"] = sorted(objects)
                    assert validity(
                        node, names, seed
                    ).valid, "Known mock seed is no longer collision-free"
                    below_floor = seed.copy()
                    below_floor[1] = 1.5707963267948966
                    collision = validity(node, names, below_floor)
                    assert not collision.valid, "Floor collision not rejected"
                    contacts = sorted(
                        {
                            body
                            for contact in collision.contacts
                            for body in (contact.contact_body_1, contact.contact_body_2)
                        }
                    )
                    assert "iimt_workspace_floor" in contacts, contacts
                    report["floor_collision_contacts"] = contacts
                # Fixture initialization ONLY: mock zero extends outside the workspace.
                # Bypass planning solely to seed fake joints, after mock/IP guards above.
                report["seed_duration_s"] = execute(
                    node,
                    FollowJointTrajectory,
                    action,
                    goal(node, names, measured(), seed, 8.0),
                    15,
                )
                spin_until(
                    node,
                    lambda: max(abs(a - b) for a, b in zip(measured(), seed)) < 0.003,
                )
                trajectory = csv_goal(node, names, rows, csv_arms)
                target = list(trajectory.trajectory.points[-1].positions)
                nominal = (
                    trajectory.trajectory.header.stamp.sec * 10**9
                    + trajectory.trajectory.header.stamp.nanosec
                )
                duration = float(rows[-1]["t"])
                report["csv_points"] = len(rows)
                report["csv_duration_s"] = duration
                report["csv_positions_only"] = True
                report["csv_roundtrip_s"] = execute(
                    node, FollowJointTrajectory, action, trajectory, duration + 8.0
                )
                completed_elapsed = (node.get_clock().now().nanoseconds - nominal) / 1e9
                report["csv_completion_from_nominal_start_s"] = completed_elapsed
                assert (
                    duration - 0.05 <= completed_elapsed <= duration + 0.5
                ), f"CSV timeline changed: source={duration}s, completed={completed_elapsed}s"
                spin_until(
                    node,
                    lambda: max(abs(a - b) for a, b in zip(measured(), target)) < 0.003,
                )
                spin_until(
                    node,
                    lambda: all(
                        timing.get(arm, {}).get("nominal_start_stamp_ns")
                        == str(nominal)
                        for arm in arms
                    ),
                )
                report["shared_nominal_start_ns"] = nominal
                report["csv_max_final_error_rad"] = max(
                    abs(a - b) for a, b in zip(measured(), target)
                )
                if dual:
                    final = target.copy()
                    final[0] -= 0.015
                    final[6] += 0.015
                    request = GetMotionPlan.Request()
                    planning = request.motion_plan_request
                    planning.group_name = "both_arms"
                    planning.pipeline_id = ""  # Use the configured default pipeline (named move_group in Humble).
                    planning.planner_id = "RRTConnect"
                    planning.allowed_planning_time = 5.0
                    planning.num_planning_attempts = 1
                    planning.max_velocity_scaling_factor = 0.1
                    planning.max_acceleration_scaling_factor = 0.1
                    planning.start_state = state(names, measured())
                    planning.goal_constraints = [
                        Constraints(
                            joint_constraints=[
                                JointConstraint(
                                    joint_name=name,
                                    position=value,
                                    tolerance_above=0.0001,
                                    tolerance_below=0.0001,
                                    weight=1.0,
                                )
                                for name, value in zip(names, final)
                            ]
                        )
                    ]
                    planned = call(
                        node, GetMotionPlan, "/plan_kinematic_path", request
                    ).motion_plan_response
                    assert planned.error_code.val == 1, planned.error_code
                    path = planned.trajectory.joint_trajectory
                    assert len(path.points) >= 2
                    for point in path.points:
                        assert validity(
                            node, path.joint_names, list(point.positions)
                        ).valid, "Planner returned colliding waypoint"
                    report["rrtconnect_waypoints"] = len(path.points)
                    report["moveit_roundtrip_s"] = execute(
                        node,
                        ExecuteTrajectory,
                        "/execute_trajectory",
                        ExecuteTrajectory.Goal(trajectory=planned.trajectory),
                        20,
                    )
                    spin_until(
                        node,
                        lambda: max(abs(a - b) for a, b in zip(measured(), final))
                        < 0.003,
                    )
                    report["moveit_max_final_error_rad"] = max(
                        abs(a - b) for a, b in zip(measured(), final)
                    )
                report["outcome"] = "PASS"
                print(json.dumps(report, indent=2), flush=True)
            except BaseException:
                print(log_path.read_text(), flush=True)
                raise
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=5)


if __name__ == "__main__":
    for mode in ("single", "dual"):
        check(mode)
