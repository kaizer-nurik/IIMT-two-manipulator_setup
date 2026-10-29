# Two direct IIMT CI-05 drivers

Rebuilt around the tested single-arm `iimt_ros2_driver`, not legacy ros2_control.
No SDK/controller implementation is duplicated here.

```bash
ros2 launch iimt_dual_control dual_control.launch.py
# Real hardware requires two explicitly different IPs; motion is opt-in:
ros2 launch iimt_dual_control dual_control.launch.py \
  use_mock_hardware:=false enable_motion:=true \
  left_robot_ip:=192.168.50.95 right_robot_ip:=192.168.50.92
```

The example IPs are not a statement about your actual wiring. `robot_type=5` only.
`left_parameters_file` / `right_parameters_file` select the arm profiles.
Per-arm overrides: `{left,right}_command_rate` (125 Hz), `_read_rate` (125 Hz),
`_feedback_transport` (`tcp`), `_read_timeout` (0.1 s),
`_trajectory_advance_ms` (0, not assumed identical between robots).
No experimental SDK TCP_NODELAY interposer is enabled.

`/both_arms_controller/follow_joint_trajectory` accepts all twelve prefixed joints.
It also accepts one complete arm's six joints, holding the other arm at fresh,
measured stationary positions. Arbitrary partial goals are rejected. Position-only
CSV points are preserved without invented derivatives or resampling.
Both child goals receive the identical absolute start timestamp. A caller's nonzero
header stamp is preserved exactly; zero stamps (MoveIt) become now +
`dispatch_lead_time_s` (1 s), requiring the largest advance plus
`dispatch_margin_s` (0.2 s). This is synchronized scheduling, **not hard realtime**.

Driver nodes are `/left/iimt_ros2_driver` and `/right/iimt_ros2_driver`.
Child actions are `/{left,right}/iimt_group_controller/follow_joint_trajectory`.
Each arm's `/joint_states` message is forwarded to global `/joint_states` with
its original header; no fictitious merged measurement timestamp is created.
The combined action has no synthesized feedback; record the per-arm joint states
and driver `trajectory_timing` topics for measurement.

A child rejection/failure cancels the other child. Parent cancellation cancels both.
The coordinator waits for terminal child results before accepting another goal;
unconfirmed stop latches a fault requiring restart **after physical stop verification**.
Both acceptances must settle before the earliest arm pre-roll minus the dispatch
margin; otherwise already accepted goals are canceled. This cannot provide atomic
distributed start if a server/network stalls beyond the margin. Late acceptance is
canceled and uncertain ownership latches a fault. Completion requires successful results from both arms.
Fresh healthy diagnostics from both drivers are required before accepting a goal.
Use only the combined action when running this stack: ROS does not prevent another
client from submitting directly to a child action. ROS cancellation, process-exit
shutdown and these software guards are **not an emergency stop or a safety PLC**.
Loss of power/network/coordinator can prevent a cancellation from being delivered.
Maintain an independent physical stop mechanism.

Description/base transforms and MoveIt live in the separate dual description and
MoveIt packages. The control launch intentionally does not publish robot_description.

## Error context

Child errors name the arm/action and preserve the driver result, including primary
cause, measured joint snapshot and separate Stop/Clear failures. Failed sibling
results during cancellation are also included; normal cancellation is not a fault.
See [diagnostic fields and timestamp semantics](../iimt_ros2_driver/ERROR_DIAGNOSTICS_RU.md).
