#!/usr/bin/env python3
"""Exercise real ROS action/error paths against test-only, network-free SDK symbols."""

import os
import re
import signal
import subprocess
import sys
import tempfile
import time
import unittest

# Deliberately independent of the user's running launch/domain.
os.environ["ROS_DOMAIN_ID"] = "219"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

EXECUTABLE = sys.argv.pop(1)
NAMES = [
    "right_" + name
    for name in (
        "shoulder_joint",
        "upper_arm_joint",
        "forearm_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )
]


class DriverErrorsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def wait(self, future, node, timeout=8.0):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), "ROS action future timed out")
        return future.result()

    def run_case(
        self, name, fixture, *, path_tolerance=3.0, target=0.1, start=0.0, cancel=False
    ):
        namespace = f"/driver_error_test_{os.getpid()}_{name}"
        environment = os.environ.copy()
        for key in ("STOP_FAIL", "CLEAR_FAIL", "SERVO_FAIL", "FOLLOW_COMMAND"):
            environment[f"IIMT_TEST_{key}"] = str(int(key in fixture))
        command = [EXECUTABLE, "--ros-args", "-r", f"__ns:={namespace}"]
        for key, value in {
            "joint_prefix": "right_",
            "robot_ip": "127.0.0.1",
            "enable_motion": "true",
            "use_mock_hardware": "false",
            "feedback_transport": "sdk",
            "command_rate": "100.0",
            "read_rate": "100.0",
            "state_timeout": "1.0",
            "max_lateness": "0.3",
            "path_tolerance": path_tolerance,
            "goal_tolerance": "0.001",
            "goal_time_tolerance": "0.2",
        }.items():
            command.extend(["-p", f"{key}:={value}"])
        node = rclpy.create_node("client", namespace=namespace)
        samples = []
        subscription = node.create_subscription(
            JointState, "joint_states", samples.append, qos_profile_sensor_data
        )
        client = ActionClient(
            node, FollowJointTrajectory, "iimt_group_controller/follow_joint_trajectory"
        )
        result = None
        with tempfile.TemporaryFile(mode="w+") as log:
            process = subprocess.Popen(
                command, env=environment, stdout=log, stderr=subprocess.STDOUT
            )
            try:
                self.assertTrue(
                    client.wait_for_server(timeout_sec=8), "No driver action server"
                )
                deadline = time.monotonic() + 5
                while not samples and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                self.assertTrue(samples, "Driver did not publish fake SDK state")
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = NAMES
                for t, q in ((0, start), (0.8, target)):
                    point = JointTrajectoryPoint()
                    point.positions = [q, 0.0, 0.0, 0.0, 0.0, 0.0]
                    point.time_from_start.sec = int(t)
                    point.time_from_start.nanosec = round((t - int(t)) * 1e9)
                    goal.trajectory.points.append(point)
                handle = self.wait(client.send_goal_async(goal), node)
                if handle.accepted:
                    if cancel:
                        # Let at least a few commands execute before requesting stop.
                        until = time.monotonic() + 0.15
                        while time.monotonic() < until:
                            rclpy.spin_once(node, timeout_sec=0.01)
                        canceled = self.wait(handle.cancel_goal_async(), node)
                        self.assertEqual(len(canceled.goals_canceling), 1)
                    result = self.wait(handle.get_result_async(), node)
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
                client.destroy()
                node.destroy_subscription(subscription)
                node.destroy_node()
            log.seek(0)
            output = log.read()
        self.assertEqual(process.returncode, 0, output)
        return result, output

    def assert_snapshot(self, message):
        for field in (
            "arm=right",
            "feedback_stamp_ns=",
            "feedback_age_s=",
            "feedback_fresh=",
        ):
            self.assertIn(field, message)
        for joint in NAMES:
            self.assertIn(joint, message)

    def assert_joint_error(self, message):
        for field in (
            "joint=right_shoulder_joint",
            "sdk_axis=1",
            "desired_rad=",
            "actual_rad=",
            "error_rad=",
            "abs_error_rad=",
            "tolerance_rad=",
        ):
            self.assertIn(field, message)

    def joint_values(self, message):
        match = re.search(
            r"joint=right_shoulder_joint sdk_axis=1 desired_rad=([-+0-9.e]+) "
            r"actual_rad=([-+0-9.e]+) error_rad=([-+0-9.e]+) "
            r"abs_error_rad=([-+0-9.e]+)(?: tolerance_rad=([-+0-9.e]+))?",
            message,
        )
        self.assertIsNotNone(match, message)
        return tuple(
            float(value) if value is not None else None for value in match.groups()
        )

    def test_path_failure_preserved_when_stop_fails(self):
        response, _ = self.run_case("path", {"STOP_FAIL"}, path_tolerance=0.015)
        self.assertIsNotNone(response)
        self.assertEqual(response.status, 6)  # STATUS_ABORTED
        message = response.result.error_string
        for field in (
            "original_code=-4",
            "original_reason=",
            "Path",
            "stop_failure=",
            "Stop=false",
            "Clear=true",
            "physical stop is NOT confirmed",
        ):
            self.assertIn(field, message)
        self.assert_joint_error(message)
        self.assert_snapshot(message)
        desired, actual, error, absolute, tolerance = self.joint_values(message)
        self.assertEqual(actual, 0.0)
        self.assertGreater(desired, actual)
        self.assertAlmostEqual(error, desired - actual, places=6)
        self.assertAlmostEqual(absolute, abs(error), places=6)
        self.assertAlmostEqual(tolerance, 0.015, places=6)
        self.assertGreater(absolute, tolerance)

    def test_goal_failure_preserved_when_clear_fails(self):
        response, _ = self.run_case("goal", {"CLEAR_FAIL"})
        self.assertIsNotNone(response)
        self.assertEqual(response.status, 6)
        message = response.result.error_string
        for field in (
            "original_code=-5",
            "original_reason=",
            "Stop=true",
            "Clear=false",
        ):
            self.assertIn(field, message)
        self.assert_joint_error(message)
        self.assert_snapshot(message)

    def test_successful_primary_does_not_invent_failed_joint(self):
        response, _ = self.run_case("stop", {"STOP_FAIL"}, target=0.0)
        self.assertIsNotNone(response)
        self.assertEqual(response.status, 6)
        message = response.result.error_string
        for field in (
            "original_code=0",
            "Stop=false",
            "joint_cause=not_reported_by_sdk",
        ):
            self.assertIn(field, message)
        self.assert_snapshot(message)
        desired, actual, error, absolute, _ = self.joint_values(message)
        self.assertEqual((desired, actual, error, absolute), (0.0, 0.0, 0.0, 0.0))

    def test_start_mismatch_rejection_logs_angles(self):
        response, output = self.run_case("start", set(), start=0.1)
        self.assertIsNone(response)
        self.assertIn("Reject trajectory", output)
        self.assertIn("Start position mismatch", output)
        self.assert_joint_error(output)
        self.assertIn("arm=right", output)

    def test_servoj_failure_has_snapshot_not_invented_joint_cause(self):
        response, _ = self.run_case("servo", {"SERVO_FAIL"})
        self.assertIsNotNone(response)
        self.assertEqual(response.status, 6)
        message = response.result.error_string
        self.assertIn("API_Servoj failed", message)
        self.assertIn("joint_cause=not_reported_by_sdk", message)
        self.assert_snapshot(message)

    def test_healthy_following_sdk_succeeds(self):
        response, _ = self.run_case("healthy", {"FOLLOW_COMMAND"})
        self.assertIsNotNone(response)
        self.assertEqual(response.status, 4)  # STATUS_SUCCEEDED
        self.assertEqual(response.result.error_code, 0)
        self.assertNotIn("stop_failure=", response.result.error_string)
        self.assertNotIn("physical stop is NOT confirmed", response.result.error_string)

    def test_healthy_cancel_retains_canceled_status(self):
        response, _ = self.run_case("cancel", {"FOLLOW_COMMAND"}, cancel=True)
        self.assertIsNotNone(response)
        self.assertEqual(response.status, 5)  # STATUS_CANCELED
        self.assertEqual(response.result.error_code, 0)
        self.assertIn("Canceled", response.result.error_string)
        self.assertNotIn("stop_failure=", response.result.error_string)


if __name__ == "__main__":
    unittest.main()
