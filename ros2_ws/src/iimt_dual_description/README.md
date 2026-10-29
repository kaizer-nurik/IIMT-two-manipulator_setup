# Dual CI-05 geometry

Two prefixed copies of the verified single-arm `iimt5.urdf`: identical meshes,
origins, inertias and limits, without `ros2_control` hardware blocks.

Xacro arguments (metres/radians, three space-separated numbers):
- `left_base_xyz`, `left_base_rpy`: left base pose **in world**.
- `right_base_xyz`, `right_base_rpy`: right base pose **in left_base_link**.

The transform is `T_world_right = T_world_left * T_left_right`.
Defaults are extracted from workspace `robot.urdf` (`left_root_joint` and
`right_root_joint`), identifying our `world` with its `root`:

| Transform | xyz (m) | rpy (rad) |
| --- | --- | --- |
| root → left | -0.0008297 -0.5380000 0.0818739 | 0 0 -0.6981317008 |
| root → right | -0.0008297 -0.0005900 -0.0001261 | 0 0 -0.6981317008 |
| left → right | -0.345440489323 0.411679944176 -0.082000000000 | 0 0 0 |

`T_left_right = inverse(T_root_left) * T_root_right`. The translation difference
must be rotated into the left base frame, not copied as a world-frame offset.
Both bases have the same -40° yaw, so their relative rotation is zero.
These are model values, not a new physical calibration. All four launch/xacro
arguments remain overridable; source `robot.urdf` and arm dynamics are unchanged.
The fixed massless `world` root avoids assigning root-link inertia to KDL.

Regression, after building and sourcing the workspace:
`python3 src/iimt_robot_ros2-humble/iimt_dual_description/test/geometry_test.py`
