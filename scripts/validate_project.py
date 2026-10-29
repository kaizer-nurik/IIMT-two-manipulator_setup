#!/usr/bin/env python3
"""Static notebook validation and ROS import smoke test; never connect to robots."""
from pathlib import Path
import ast
import nbformat
import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory

root = Path(__file__).resolve().parents[1]
for path in sorted((root / "notebooks").glob("*.ipynb")):
    notebook = nbformat.read(path, as_version=4)
    nbformat.validate(notebook)
    for index, cell in enumerate(notebook.cells):
        if cell.cell_type == "code":
            ast.parse(cell.source, filename=f"{path.name}:cell{index}")
    print(f"Validated {path.name} (not executed)")
print("ROS imports OK; no robot connection or notebook execution")
