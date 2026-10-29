# Offline control-core checks

No ROS installation, vendor library, network connection, or robot is used.
The transport check loads the local fake SDK, not the real vendor SDK.

From the workspace directory containing `ros2_ws`:

```bash
cmake -S ros2_ws/src/iimt_robot_ros2-humble/iimt_ros2_control/test -B /tmp/iimt-core-tests
cmake --build /tmp/iimt-core-tests
ctest --test-dir /tmp/iimt-core-tests --output-on-failure
```

`control_core_test.cpp` checks configuration, all six joints, finite values,
position/step/velocity/acceleration/tracking bounds, watchdog timestamps,
latest-only command storage, absolute polling-grid timing and missed-slot skipping.
The four-second timing case is arithmetic, not a wall-clock benchmark.

`transport_test.cpp` compiles the production SDK adapter against the vendor
headers and verifies it through `fake_sdk.cpp`: parameter forwarding, independent
connections, feedback, failed SDK calls, stop/clear, and connection cleanup.

Optional sanitizer run:

```bash
cmake -S ros2_ws/src/iimt_robot_ros2-humble/iimt_ros2_control/test \
  -B /tmp/iimt-core-tests-sanitize \
  -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
  -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=address,undefined' \
  -DCMAKE_SHARED_LINKER_FLAGS='-fsanitize=address,undefined'
cmake --build /tmp/iimt-core-tests-sanitize
ctest --test-dir /tmp/iimt-core-tests-sanitize --output-on-failure
```

These standalone checks do not prove ROS lifecycle behavior, controller-manager
integration or worker-thread fault latching. The conditional ROS suite below
checks lifecycle and worker behavior with a fake SDK. Neither suite establishes
real SDK call latency, hard real-time guarantees, firmware ServoJ semantics or
physical tracking accuracy; those require controlled hardware validation.

When built from the ROS package, `system_test.cpp` is also enabled and links the
real `IimtSystem` class with the fake SDK. It exercises physical axis-order
validation, activation seeding,
read-only safety, interface claiming, unchanged-position heartbeats, controller
and SDK failure latching, prompt `on_error`, diagnostic-node retention, and
cleanup. It also sends a smooth 0.2 rad, two-second quintic move through the
real worker with fake feedback following successful commands. The independent
writer has a nominal 20 ms period and bounded +/-4 ms phase modulation
(`command_jitter_tolerance=0.01` in this 20 ms fixture); reference
positions are evaluated at actual elapsed time, not incremented per tick. Peak
reference velocity (0.1875 rad/s) and acceleration (<0.289 rad/s^2) are below
unchanged hardware guard defaults. This regression detects repeated/skipped
mailbox samples that turn a smooth reference into discontinuous emitted velocity.
A second moving case uses the target 125 Hz rate (8 ms), +/-1 ms writer
phase modulation, and the default 4 ms jitter tolerance. The suite also verifies
one send per write without periodic replay, rejection of bunched writes,
reclaim blocked until asynchronous Stop completes, and reported/retryable SDK
logout failure. These are bounded host checks, not a real-time guarantee.
Real-robot tracking and SDK latency remain separate hardware acceptance checks.


Initial validation in ROS 2 Humble / Ubuntu 22.04 passed the six hold/lifecycle
cases, but did not establish moving-trajectory behavior. The added moving
regression initially failed the independently clocked repeat-send worker at
0.416 s with an emitted acceleration guard fault (six cases passed, one failed).
This was a functional scheduling regression, not evidence to increase the guard
limits. After the command-driven scheduler correction, all three CTest targets
passed, including 12 real-class lifecycle/worker cases (6.00 s). Both 50 Hz and
125 Hz moving cases passed with unchanged acceleration limits. A subsequent
repeated 125 Hz run exposed inconsistent dispatch timestamp accounting; that
was corrected without changing guard limits. The final exact revision passed
all three CTest targets plus three consecutive runs of the 12-case lifecycle
suite (18.02 s total). This is bounded qualification, not a guarantee on a busy
non-real-time host.
No real SDK library or robot is used by these tests.

Optional full MoveIt mock launch smoke check (not automatically run by CTest):

```bash
python3 ros2_ws/src/iimt_robot_ros2-humble/iimt_ros2_control/test/moveit_smoke.py
```

It queries the *running* controller-manager and refuses to send a goal unless
all hardware `class_type` values equal `mock_components/GenericSystem` exactly.
It sends a two-second, four-waypoint rest-to-rest closed-loop trajectory via
MoveIt's `/execute_trajectory`, checks actual JTC state excursion, same-message
reference/feedback error, monotonic state timestamps and action timing. Because
start and endpoint coincide, immediate endpoint-only success is rejected.

Dual-instance regressions use independent fake robot states keyed by IP and
unique SDK handles. The legacy first-robot test helpers remain supported. The
suite checks prefixed six-axis measurement/command routing, distinct parameter
nodes, peer-fault latching, and serialization across all vendor API entries.
A delayed second robot Login exercises startup contention without faulting an
idle first robot. A condition-variable-blocked feedback call exercises armed
SDK gate timeout: both arms must latch, attempt Stop after the gate reopens,
and never replay pending commands. No physical robot or network SDK is used.

Dual regression validation (ROS 2 Humble / Ubuntu 22.04, fake SDK): the three
CTest targets passed, including all 16 lifecycle/worker cases. Three additional
consecutive runs of the lifecycle suite passed (19.38 seconds total). Standalone
core/transport CTest passed 2/2. This verifies software isolation and failure
behavior, not simultaneous physical ServoJ arrival or two-robot tracking error.
The new SDK gate timeout is included in finite/positive/clock-representation
configuration boundary tests.
A subsequent full-suite run passed all dual cases but failed the existing 125 Hz
moving case during its initial live-hold setup: the hardware correctly latched
"controller command deadline missed" (15/16 cases passed). Thus the suite remains
host-scheduling-sensitive; the earlier repeat passes are bounded evidence, not
qualification of a non-real-time host. Timing guards were not relaxed.

Focused dual validation can be run independently of host-sensitive timing cases:

```bash
build/iimt_ros2_control/test/iimt_system_test \
  build/iimt_ros2_control/test/libiimt_fake_sdk.so dual
```

The optional final argument filters case names; selecting zero cases fails.
Latest focused run: **4 passed, 0 failed**. Default CTest still selects all cases.

The additional `dual 125Hz moving paths` case sends distinct smooth two-second
quintic trajectories to all twelve axes through two real IimtSystem instances.
Both writes use one common source timestamp; fake feedback follows each robot's
actual successful ServoJ commands independently by IP. The test requires exactly
275 sends per robot, both distinct endpoints, no Stop/fault, and no overlapping
vendor calls. Displacements are 0.10–0.12 rad, with unchanged hardware guard limits.
It is a host-sensitive integration qualification, not proof of physical arrival
synchronization. Validation for this newly added case is pending the next build.

Final dual extension validation: seven-package Humble build passed; CTest 3/3
including 17 lifecycle cases passed (8.69s), focused five dual cases passed 5/5.
The new simultaneous 125 Hz moving case used distinct paths on all 12 axes, common
source timestamps and exactly 275 sends per robot. ASan/UBSan core/transport passed 2/2.
Earlier non-RT host timing failures above remain qualification limitations.

SDK-budget startup regressions (filter `"SDK budget"`) inject a 15 ms successful
feedback read with a 10 ms motion budget and a 50 ms cycle. Unclaimed read-only and
motion-enabled/preclaim instances must remain observable without ServoJ/Stop/Clear;
the same delay after command claim must still fault with a live controller
heartbeat and request Stop. A failed unclaimed SDK read remains fatal rather than
being mistaken for a timing warning. Delays run outside the fake-state mutex so
observers are not artificially blocked. Baseline before the production fix:
**2 passed, 1 failed**; the expected failing case was unclaimed `on_configure`.

The unclaimed case also checks the published WARN status and per-operation
read-duration/read-overrun counters, with zero ServoJ calls. A controlled blocked
read followed by command claim verifies that a call started while idle cannot
escape armed enforcement when it returns after the claim.

SDK read-only timing correction (2026-10-26): delayed 15 ms read against 10 ms
motion budget reproduced the original unclaimed startup failure (RED 2/3).
Final focused tests passed 4/4: observable unclaimed reads plus WARN/metrics,
strict armed reads, claim acquired during a blocked read, and fatal SDK read
failure. Final Humble build passed and CTest 3/3 includes 21 lifecycle cases
(9.22s). An earlier full run hit the dual 125 Hz acceleration guard (19/20);
that failed qualification evidence was not erased by the later pass. No real
SDK/robot operations were used; hardware 125 Hz timing remains unqualified.
