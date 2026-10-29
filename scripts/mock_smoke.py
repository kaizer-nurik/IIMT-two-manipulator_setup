#!/usr/bin/env python3
"""Check both launch graphs with fake hardware only; send no motion goals."""
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPlanningScene
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def wait_future(node, future, timeout=10):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError("ROS request timed out")
    return future.result()


def check(mode):
    package, launch, drivers, action = (
        ("iimt_ros2_moveit_config", "iimt_driver_moveit.launch.py",
         ["/iimt_ros2_driver"], "/iimt_group_controller/follow_joint_trajectory")
        if mode == "single" else
        ("iimt_dual_moveit_config", "dual_moveit.launch.py",
         ["/left/iimt_ros2_driver", "/right/iimt_ros2_driver"],
         "/both_arms_controller/follow_joint_trajectory")
    )
    command = ["ros2", "launch", package, launch, "use_mock_hardware:=true",
               "enable_motion:=false", "launch_rviz:=false"]
    log_path = Path(tempfile.gettempdir()) / f"iimt-{mode}-mock-smoke.log"
    with log_path.open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        rclpy.init()
        node = rclpy.create_node("iimt_packaging_smoke")
        try:
            for driver in drivers:
                service = node.create_client(GetParameters, driver + "/get_parameters")
                if not service.wait_for_service(timeout_sec=30):
                    raise RuntimeError(f"{driver} absent; see {log_path}")
                response = wait_future(node, service.call_async(GetParameters.Request(
                    names=["use_mock_hardware", "enable_motion"])))
                assert response.values[0].bool_value and not response.values[1].bool_value
                node.destroy_client(service)
            client = ActionClient(node, FollowJointTrajectory, action)
            assert client.wait_for_server(timeout_sec=15), action
            samples = {}
            topics = ["/joint_states"] if mode == "single" else ["/left/joint_states", "/right/joint_states"]
            subscriptions = [node.create_subscription(
                JointState, topic, lambda message, topic=topic: samples.update({topic: message}),
                qos_profile_sensor_data) for topic in topics]
            deadline = time.monotonic() + 10
            while len(samples) < len(topics) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            assert all(topic in samples and len(samples[topic].position) == 6 for topic in topics)
            if mode == "dual":
                service = node.create_client(GetPlanningScene, "/get_planning_scene")
                assert service.wait_for_service(timeout_sec=15)
                deadline = time.monotonic() + 15
                while True:
                    request = GetPlanningScene.Request()
                    request.components.components = 16  # WORLD_OBJECT_GEOMETRY
                    scene = wait_future(node, service.call_async(request)).scene
                    objects = {obj.id for obj in scene.world.collision_objects}
                    if "iimt_workspace_floor" in objects and len(objects) >= 5:
                        break
                    if time.monotonic() > deadline:
                        raise RuntimeError("Workspace/floor missing")
                    time.sleep(.2)
                node.destroy_client(service)
            client.destroy()
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            print(f"{mode}: mock=true, motion=false, action/joint states/scene OK", flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)


if __name__ == "__main__":
    # Isolate the test graph from normal default-domain robot deployments.
    os.environ["ROS_DOMAIN_ID"] = os.environ.get("IIMT_TEST_DOMAIN_ID", "217")
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    for mode in ("single", "dual"):
        check(mode)
