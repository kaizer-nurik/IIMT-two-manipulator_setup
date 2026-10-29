#!/usr/bin/env python3
"""Run the actual CSV loader cells and 2D plotting in the registered notebook kernel.

Only config/import/CSV cells are executed, never ROS setup, planning or motion cells.
Run from the project root or notebooks directory; no old workspace is needed.
"""

from pathlib import Path

from jupyter_client import KernelManager

ROOT = Path(__file__).resolve().parents[1]
CODE = r"""
%matplotlib inline
from pathlib import Path
import json
import rclpy
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from control_msgs.action import FollowJointTrajectory
import numpy as np
import matplotlib.pyplot as plt
for name, indices, joints in (
    ("iimt_ci05_left_csv_ethernet.ipynb", (2, 3), 6),
    ("iimt_ci05_dual_csv_ethernet.ipynb", (1, 3), 12),
):
    notebook = json.loads((Path(PROJECT_ROOT) / "notebooks" / name).read_text())
    environment = {"display": lambda value: None}
    for index in indices:
        exec("".join(notebook["cells"][index]["source"]), environment)
    assert environment["ROOT"] == Path(PROJECT_ROOT), environment["ROOT"]
    q, t = environment["q"], environment["t"]
    assert q.shape == (len(t), joints) and len(t) >= 2
    assert np.isfinite(q).all() and np.all(np.diff(t) > 0)
    plt.figure()
    plt.plot(t, q[:, 0])
    plt.title(name + " CSV loader (no robot measurement)")
    plt.show()
    plt.close("all")
    print(name, q.shape, "CSV + inline plot PASS")
""".replace("PROJECT_ROOT", repr(str(ROOT)))
manager = KernelManager(kernel_name="iimt-ros2")
manager.start_kernel(cwd=str(ROOT / "notebooks"))
client = manager.client()
client.start_channels()
try:
    client.wait_for_ready(timeout=30)
    request = client.execute(CODE.strip())
    images = 0
    while True:
        message = client.get_iopub_msg(timeout=30)
        if message["parent_header"].get("msg_id") != request:
            continue
        content = message["content"]
        if message["msg_type"] == "error":
            raise RuntimeError("\n".join(content["traceback"]))
        if message["msg_type"] == "stream":
            print(content["text"], end="")
        if message["msg_type"] == "display_data":
            images += int("image/png" in content["data"])
        if message["msg_type"] == "status" and content["execution_state"] == "idle":
            break
    assert images == 2, f"Expected two inline PNG plots, received {images}"
    print(
        "PASS: actual iimt-ros2 kernel, both CSV loaders, ROS imports, two inline plots"
    )
finally:
    client.stop_channels()
    manager.shutdown_kernel(now=True)
