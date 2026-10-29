#!/usr/bin/env python3
"""Check declared and actual runtime dependencies; no ROS nodes or SDK calls."""

import importlib
import subprocess
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from jupyter_client.kernelspec import KernelSpecManager

ROOT = Path(__file__).resolve().parents[1]
subprocess.run([sys.executable, "-m", "pip", "check"], check=True)
subprocess.run(
    [
        "rosdep",
        "check",
        "--from-paths",
        str(ROOT / "ros2_ws/src"),
        "--ignore-src",
        "--rosdistro",
        "humble",
    ],
    check=True,
)
for module in (
    "rclpy",
    "rpyutils.import_c_library",
    "control_msgs.action",
    "trajectory_msgs.msg",
    "moveit_msgs.action",
    "moveit_msgs.srv",
    "sensor_msgs.msg",
    "diagnostic_msgs.msg",
    "rcl_interfaces.srv",
    "builtin_interfaces.msg",
    "shape_msgs.msg",
    "geometry_msgs.msg",
    "rosidl_runtime_py.convert",
    "launch",
    "launch_ros",
    "yaml",
    "numpy",
    "pandas",
    "matplotlib.pyplot",
    "nbformat",
    "jupyterlab",
    "ipykernel",
    "openpyxl",
):
    imported = importlib.import_module(module)
    print(f"IMPORT {module}: {imported.__file__}", flush=True)
library = (
    Path(get_package_prefix("iimt_ros2_driver")) / "lib/iimt_ros2_driver/libiimtcl.so"
)
linked = subprocess.check_output(["ldd", str(library)], text=True)
print(linked)
assert "not found" not in linked, "Missing vendor SDK shared-library dependency"
kernel = KernelSpecManager().get_kernel_spec("iimt-ros2")
# Compare paths, not resolved binaries: system Python and venv symlinks may share a target.
assert Path(kernel.argv[0]) == Path(sys.executable), (kernel.argv, sys.executable)
assert sys.version_info[:2] == (
    3,
    10,
), "ROS Humble binary modules require image Python 3.10"
print(
    f"PASS: Python {sys.version.split()[0]}, kernel={kernel.argv[0]}, rosdep/pip/SDK resolved"
)
