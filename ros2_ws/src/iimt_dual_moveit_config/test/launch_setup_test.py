import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

root = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "dual_launch", root / "iimt_dual_moveit_config/launch/dual_moveit.launch.py"
)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
c = LaunchContext()
for entity in m.generate_launch_description().entities:
    if isinstance(entity, DeclareLaunchArgument):
        entity.execute(c)
description = ET.parse(
    root / "iimt_dual_description/urdf/iimt_dual.urdf.xacro"
).getroot()
for argument in description.findall("{http://www.ros.org/wiki/xacro}arg"):
    assert c.launch_configurations[argument.get("name")] == argument.get("default")
for side in ("x_min", "y_min", "x_max", "y_max"):
    assert c.launch_configurations[f"workspace_{side}_margin"] == "0.5"
# Common margin supplies defaults, a side-specific value takes precedence.
overrides = LaunchContext()
overrides.launch_configurations.update(
    workspace_margin="0.7", workspace_x_max_margin="0.9"
)
for entity in m.generate_launch_description().entities:
    if isinstance(entity, DeclareLaunchArgument):
        entity.execute(overrides)
assert overrides.launch_configurations["workspace_x_min_margin"] == "0.7"
assert overrides.launch_configurations["workspace_x_max_margin"] == "0.9"
c.launch_configurations["left_parameters_file"] = str(
    root / "iimt_dual_control/config/left_driver.yaml"
)
c.launch_configurations["right_parameters_file"] = str(
    root / "iimt_dual_control/config/right_driver.yaml"
)
c.launch_configurations["launch_rviz"] = "false"
assert c.launch_configurations["floor_z"] == "auto"
a = m.launch_setup(c)
assert len([x for x in a if isinstance(x, Node)]) == 3
p = m.planning_limits(c)["robot_description_planning"]["joint_limits"]
assert len(p) == 12
assert all(v["max_velocity"] == 1.25 for v in p.values())
c.launch_configurations["right_command_rate"] = "30"
p = m.planning_limits(c)["robot_description_planning"]["joint_limits"]
assert p["right_shoulder_joint"]["max_velocity"] == 0.3
assert p["left_shoulder_joint"]["max_velocity"] == 1.25
for name, val in [("right_base_xyz", "nan 0 0"), ("left_base_rpy", "0 0")]:
    old = c.launch_configurations[name]
    c.launch_configurations[name] = val
    try:
        m.base_transforms(c)
    except ValueError:
        pass
    else:
        raise AssertionError("Bad geometry accepted")
    c.launch_configurations[name] = old
print(
    "Launch construction, 12 profile bounds, independent rates and invalid-transform checks PASS; no nodes started"
)

for invalid_floor in ("nan", "inf", "-inf"):
    c.launch_configurations["floor_z"] = invalid_floor
    try:
        m.launch_setup(c)
    except ValueError:
        pass
    else:
        raise AssertionError("Invalid floor height accepted")
c.launch_configurations["floor_z"] = "auto"
