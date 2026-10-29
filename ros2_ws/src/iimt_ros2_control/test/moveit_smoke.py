#!/usr/bin/env python3
"""Optional MoveIt graph smoke check. Refuses every non-GenericSystem graph."""
import argparse
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint


def require(ok, why):
    if not ok:
        raise RuntimeError(why)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", default="iimt_group_controller")
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node("iimt_mock_moveit_smoke")
    client = None
    try:
        def wait(predicate, timeout=args.timeout):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if predicate():
                    return
                rclpy.spin_once(node, timeout_sec=0.02)
            require(predicate(), "ROS graph/future timeout")

        def call(service_type, name):
            service = node.create_client(service_type, name)
            require(service.wait_for_service(timeout_sec=args.timeout), f"Missing {name}")
            future = service.call_async(service_type.Request())
            wait(future.done)
            result = future.result()
            node.destroy_client(service)
            return result

        # Fail closed before constructing or sending any action goal. Merely passing a
        # mock launch argument is insufficient evidence of the running hardware type.
        hardware = call(ListHardwareComponents, "/controller_manager/list_hardware_components")
        require(bool(hardware.component), "No hardware components reported")
        require(all(c.class_type == "mock_components/GenericSystem" for c in hardware.component),
                "REFUSED: every hardware class_type must be mock_components/GenericSystem")
        controllers = call(ListControllers, "/controller_manager/list_controllers")
        require(any(c.name == args.controller and c.state == "active" and
                    c.type == "joint_trajectory_controller/JointTrajectoryController"
                    for c in controllers.controller), "Expected active JTC not found")

        samples = []

        def on_state(message):
            desired = message.reference if message.reference.positions else message.desired
            actual = message.feedback if message.feedback.positions else message.actual
            if desired.positions and actual.positions:
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                samples.append((time.monotonic(), stamp, list(message.joint_names),
                                list(desired.positions), list(actual.positions)))

        node.create_subscription(JointTrajectoryControllerState,
                                 f"/{args.controller}/controller_state", on_state, 100)
        wait(lambda: bool(samples))
        names, base = samples[-1][2], samples[-1][4]
        require(len(names) == 6 and len(base) == 6 and all(map(math.isfinite, base)),
                "Need six finite measured joints")
        goal = ExecuteTrajectory.Goal()
        trajectory = goal.trajectory.joint_trajectory
        trajectory.joint_names = names
        # Closed loop: endpoint equals measured start. Early success based only on
        # endpoint proximity would incorrectly skip the clearly observable excursion.
        for seconds, offset in [(0.0, 0.0), (2 / 3, 0.02), (4 / 3, -0.02), (2.0, 0.0)]:
            point = JointTrajectoryPoint()
            point.positions = list(base)
            point.positions[0] += offset
            point.velocities = [0.0] * 6
            point.accelerations = [0.0] * 6
            ns = round(seconds * 1_000_000_000)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1_000_000_000)
            trajectory.points.append(point)
        client = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
        require(client.wait_for_server(timeout_sec=args.timeout), "No MoveIt execute_trajectory server")
        samples.clear()
        started = time.monotonic()
        future = client.send_goal_async(goal)
        wait(future.done)
        handle = future.result()
        require(handle.accepted, "MoveIt rejected mock trajectory")
        result_future = handle.get_result_async()
        wait(result_future.done, timeout=6.0)
        elapsed = time.monotonic() - started
        result = result_future.result()
        require(result.status == GoalStatus.STATUS_SUCCEEDED and
                result.result.error_code.val == MoveItErrorCodes.SUCCESS,
                f"MoveIt execution failed: status {result.status}, code {result.result.error_code.val}")
        require(1.6 <= elapsed <= 2.4, f"Execution timing {elapsed:.3f}s, expected 2.0±0.4s")
        require(len(samples) >= 20, "Insufficient controller state evidence")
        require(all(s[2] == names and len(s[3]) == 6 and len(s[4]) == 6 for s in samples),
                "Controller state joint order changed")
        require(all(math.isfinite(value) for s in samples for value in s[3] + s[4]),
                "Nonfinite controller state")
        peak_error = max(abs(d - a) for s in samples for d, a in zip(s[3], s[4]))
        excursion = max(abs(s[4][0] - base[0]) for s in samples)
        require(excursion >= 0.015, "No closed-loop excursion observed: possible premature success")
        require(peak_error <= 0.005, f"Mock same-stamp tracking error {peak_error:.6f} rad")
        require(all(b[1] >= a[1] for a, b in zip(samples, samples[1:])), "Controller timestamps reversed")
        print(json.dumps({"hardware": [c.class_type for c in hardware.component],
                          "controller": args.controller, "elapsed_s": elapsed,
                          "samples": len(samples), "max_same_stamp_error_rad": peak_error,
                          "observed_excursion_rad": excursion,
                          "result": "PASS (mock graph only; not physical tracking evidence)"}, indent=2))
    finally:
        if client is not None:
            client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
