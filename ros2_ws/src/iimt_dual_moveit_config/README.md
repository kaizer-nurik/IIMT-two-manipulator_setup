# Dual CI-05 MoveIt with direct SDK drivers

```bash
ros2 launch iimt_dual_moveit_config dual_moveit.launch.py \
  use_mock_hardware:=true enable_motion:=false \
  left_base_xyz:="-0.0008297 -0.5380000 0.0818739" left_base_rpy:="0 0 -0.6981317008" \
  right_base_xyz:="-0.345440489323 0.411679944176 -0.082000000000" right_base_rpy:="0 0 0"
```

Right base pose is **relative to the left base**, not world. Defaults are extracted from workspace `robot.urdf` (`world` corresponds to its
`root`). Verify that this model matches the real mounting before collision planning.

The launch includes `iimt_dual_control/dual_control.launch.py`: two independent
instances of the successful single-arm SDK driver and one synchronized
`/both_arms_controller/follow_joint_trajectory` coordinator. No ros2_control,
controller_manager or old hardware plugin is involved.

Use `both_arms` for dual joint-space planning/execution. `left_arm` and `right_arm`
have separate KDL solvers; the composite group intentionally has no chain IK
solver. Single-arm goals use the same coordinator and hold the other arm at its
fresh stationary measured pose; arbitrary partial joint sets are rejected.
No cross-arm collision pairs are disabled. The SRDF preserves only the
single-arm exclusions separately for each copy.

Each arm has its own `left_*` / `right_*` arguments: `robot_ip`, `parameters_file`,
`command_rate`, `read_rate`, `read_timeout`, `feedback_transport`,
`trajectory_advance_ms`. Real mode needs two distinct actual IP addresses,
`use_mock_hardware:=false enable_motion:=true`. Do not copy a measured advance
from one arm onto another without measurement. Default advances are zero.

Profiles supply per-joint bounds. MoveIt planning velocity is
`planning_limit_scale * min(max_velocity, max_step * command_rate)` and planning
acceleration is `planning_limit_scale * max_acceleration` (default scale0.5).
These planning margins do not weaken driver safety checks. Direct CSV action
execution bypasses MoveIt; the dual notebook now checks its linear path with
MoveIt state validity before dispatch. This is discrete preflight, not live protection.

`dispatch_lead_time_s`/`dispatch_margin_s` belong to the dual synchronization
handshake. Explicit future message timestamps remain the common nominal time;
the notebook sets its own start delay in that message, not in either driver.

## Workspace side boundaries

`workspace_margin:=0.5` installs four solid collision objects in MoveIt's planning
scene. Bounds are aligned with **world X (front/back), Y (left/right)**, not with
rotated arm axes: `xmin/min(base x)-margin`, `xmax/max(base x)+margin` and likewise
for Y. The right base is first transformed from the left base frame to world.
With the supplied mount defaults: X `[-0.5008297, 0.4991703]` m, Y
`[-1.038, 0.49941]` m. These constrain **all modeled robot collision geometry**,
not only the end effectors. A pose outside this region may now be invalid.

Each side can override the shared default in metres:
`workspace_x_min_margin` (back/−X), `workspace_x_max_margin` (front/+X),
`workspace_y_min_margin` (−Y, left-arm side with current mounting),
`workspace_y_max_margin` (+Y, right-arm side with current mounting).
All inherit `workspace_margin` unless explicitly supplied. Example:
`workspace_margin:=0.5 workspace_x_max_margin:=0.6`.

No ceiling is installed. The lateral outside regions are represented by solid
10 m thick slabs, extending from `min(base z)-10` to `max(base z)+10` m, far beyond
the fixed CI05's reachable envelope. This is not a mathematical infinite
halfspace: attached objects larger than that envelope or different robot models
require revisiting this representation. Plane messages were intentionally not
used for the side boundaries: MoveIt Humble's FCL backend creates `fcl::Plane`, not `fcl::Halfspace`, so a
fully outside pose would not necessarily collide with a plane surface.

`floor_z:=auto` (default) adds the horizontal infinite plane
`iimt_workspace_floor` at the lower of the two base origins in `world`: with
the supplied transforms, z=-0.0001261 m. This default follows changed mounting
transforms automatically. An explicit finite height such as `floor_z:=-0.1`
overrides it. The node exposes the resolved numeric `floor_z` parameter; plane
coefficients are `[0, 0, 1, -floor_z]`. This is a surface,
not a filled below-floor halfspace. Only contacts between this floor and the
fixed `left_base_link` / `right_base_link` mounts are allowed. All moving links
and tools retain floor collision checks. The exception accommodates the modeled
right mount at z=-0.0001261 m without disabling arm-floor collision checking.
The node reads and merges the existing allowed collision matrix first, preserving
its other pairs/defaults; do not concurrently edit the ACM during startup.

The `iimt_workspace` node applies objects through `/apply_planning_scene`, retries
until acknowledged, then leaves the scene unchanged. It does not restore objects
if another client later deletes them. Clients must check `/get_planning_scene`
for `iimt_workspace_x_min`, `iimt_workspace_x_max`, `iimt_workspace_y_min`,
`iimt_workspace_y_max`, and `iimt_workspace_floor` before executing a plan.
Geometry, margin and floor-height parameters
are read-only; restart launch to change them. Wait for `Workspace boundaries and floor applied` before planning in RViz. Helper exit shuts down the launch;
notebook refuses missing boundaries. Direct clients bypassing the notebook still
have no collision preflight. Do not delete boundary objects or add additional ACM exclusions.

Historical check: the current CSV is rejected with margin 0.4: at scaled time 5.672 s,
`left_wrist_3_link` intersects `iimt_workspace_x_max`. The all-zero mock pose is
also outside the allowed region. Neither case is automatically bypassed or repaired.
With margin 0.5 and the lower-base floor, the same CSV passed discrete collision
preflight and full mock execution. Random RRTConnect motion and the CSV approach
also passed. Invalid moving-arm and attached-tool contacts with the floor were
rejected. Evidence: `results/dual_floor_mock.json`; this is not physical validation.
