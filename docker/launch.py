#!/usr/bin/env python3
"""Translate explicit Compose settings into launch arguments without shell eval."""
import os
import shlex
import sys


def launch_command(mode):
    common = {
        "use_mock_hardware": "USE_MOCK_HARDWARE",
        "enable_motion": "ENABLE_MOTION",
        "launch_rviz": "LAUNCH_RVIZ",
    }
    if mode == "single":
        package, launch = "iimt_ros2_moveit_config", "iimt_driver_moveit.launch.py"
        names = ("robot_ip", "command_rate", "read_rate", "read_timeout",
                 "feedback_transport", "trajectory_advance_ms")
    elif mode == "dual":
        package, launch = "iimt_dual_moveit_config", "dual_moveit.launch.py"
        names = tuple(f"{arm}_{name}" for arm in ("left", "right") for name in (
            "robot_ip", "command_rate", "read_rate", "read_timeout",
            "feedback_transport", "trajectory_advance_ms")) + (
                "left_base_xyz", "left_base_rpy", "right_base_xyz", "right_base_rpy",
                "workspace_margin", "floor_z")
    else:
        raise ValueError("mode must be single or dual")
    common.update({name: name.upper() for name in names})
    arguments = {name: os.environ[key] for name, key in common.items() if os.environ.get(key)}
    # Even a direct invocation outside Compose defaults to no physical motion.
    arguments.setdefault("use_mock_hardware", "true")
    arguments.setdefault("enable_motion", "false")
    arguments.setdefault("launch_rviz", "false")
    for token in shlex.split(os.environ.get("LAUNCH_ARGS", "")):
        if ":=" not in token:
            raise ValueError("LAUNCH_ARGS must contain ROS name:=value arguments")
        name, value = token.split(":=", 1)
        arguments[name] = value
    return ["ros2", "launch", package, launch] + [
        f"{key}:={value}" for key, value in arguments.items()]


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "single"
    command = launch_command(mode)
    if "--show-args" in sys.argv:
        # Ask ROS to list declared arguments, without constructing/running nodes.
        command.append("--show-args")
        os.execvp(command[0], command)
    print(shlex.join(command), flush=True)
    # Shared Compose volume forbids accidental concurrent single/dual stacks.
    os.execvp("flock", ["flock", "--verbose", "--nonblock", "/run/iimt/robot-launch.lock", *command])
