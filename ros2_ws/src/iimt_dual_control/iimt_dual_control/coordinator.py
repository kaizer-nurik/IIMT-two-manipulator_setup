"""One nonblocking ROS executor coordinating two existing guarded arm drivers."""

import copy
import math
import signal
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from sensor_msgs.msg import JointState

from .trajectory import ARMS, NAMES, nanoseconds, scheduled_start, split_goal, validate


def child_context(index):
    arm = ARMS[index]
    return f"arm={arm}, action=/{arm}/iimt_group_controller/follow_joint_trajectory"


def child_result(index, response):
    """Keep the driver's measured-joint context intact; never infer a joint fault here."""
    return (
        f"{child_context(index)}, status={response.status}, "
        f"error_code={response.result.error_code}: "
        f"{response.result.error_string or '(driver provided no error details)'}"
    )


def child_call(index, operation, callback, *args):
    try:
        return callback(*args)
    except Exception as error:
        raise RuntimeError(
            f"{child_context(index)}: {operation} failed: {type(error).__name__}: {error}"
        ) from error


def append_child_failures(message, results):
    """Report terminal sibling failures discovered during cleanup, without changing ownership."""
    for index, future in results.items():
        if not future.done():
            continue
        try:
            response = child_call(index, "cleanup result retrieval", future.result)
            if response.result.error_code == 0 and response.status in (
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED,
            ):
                continue
            detail = child_result(index, response)
        except Exception as error:  # noqa: BLE001
            detail = str(error)
        if detail not in message:
            message += f"; Child cleanup result: {detail}"
    return message


class Coordinator(Node):
    def __init__(self):
        super().__init__("iimt_dual_coordinator")
        defaults = {
            "dispatch_lead_time_s": 1.0,
            "dispatch_margin_s": 0.2,
            "left_trajectory_advance_ms": 0.0,
            "right_trajectory_advance_ms": 0.0,
            "acceptance_timeout_s": 2.0,
            "cancel_timeout_s": 3.0,
            "completion_margin_s": 5.0,
            "diagnostics_timeout_s": 3.0,
        }
        self.config = {
            key: self.declare_parameter(
                key, value, ParameterDescriptor(read_only=True)
            ).value
            for key, value in defaults.items()
        }
        if any(not math.isfinite(value) or value < 0 for value in self.config.values()):
            raise ValueError(
                f"Coordinator timing parameters must be finite and nonnegative; values={self.config}"
            )
        self.required = (
            self.config["dispatch_margin_s"]
            + max(self.config[f"{arm}_trajectory_advance_ms"] for arm in ARMS) / 1000
        )
        if self.config["dispatch_lead_time_s"] < self.required:
            raise ValueError(
                f"dispatch_lead_time_s={self.config['dispatch_lead_time_s']} must cover advance and dispatch margin; required={self.required} s"
            )
        self.busy = False
        self.stopping = False
        self.fault = ""
        self.health = {}
        self.states = {}
        self.group = ReentrantCallbackGroup()
        self.arm_clients = [
            ActionClient(
                self,
                FollowJointTrajectory,
                f"/{arm}/iimt_group_controller/follow_joint_trajectory",
                callback_group=self.group,
            )
            for arm in ARMS
        ]
        self.publisher = self.create_publisher(JointState, "/joint_states", 50)
        self.subscriptions_owned = []
        for arm in ARMS:
            self.subscriptions_owned.append(
                self.create_subscription(
                    JointState,
                    f"/{arm}/joint_states",
                    lambda message, arm=arm: self.update_state(arm, message),
                    qos_profile_sensor_data,
                )
            )
            self.subscriptions_owned.append(
                self.create_subscription(
                    DiagnosticArray,
                    f"/{arm}/iimt_ros2_driver/diagnostics",
                    lambda message, arm=arm: self.update_health(arm, message),
                    10,
                )
            )
        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            "/both_arms_controller/follow_joint_trajectory",
            execute_callback=self.execute,
            goal_callback=self.accept,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group,
        )

    def update_state(self, arm, message):
        self.publisher.publish(message)
        previous = self.states.get(arm)
        stationary = False
        now = self.get_clock().now().nanoseconds
        stamp = nanoseconds(message.header.stamp)
        if previous and message.name == previous[0].name:
            dt = (stamp - nanoseconds(previous[0].header.stamp)) / 1e9
            if (
                0 < dt <= 0.2
                and len(message.position) == len(previous[0].position) == 6
            ):
                stationary = all(
                    abs(a - b) / dt < 0.01
                    for a, b in zip(message.position, previous[0].position)
                )
        self.states[arm] = (message, stationary, now)

    def normalized(self, goal):
        """A complete single arm may move only while the other arm holds fresh still positions."""
        names = goal.trajectory.joint_names
        if len(names) == 12:
            return goal
        if len(names) != 6 or len(set(names)) != 6:
            raise ValueError(
                f"Only complete six- or twelve-joint goals are supported; received joint_names={list(names)}"
            )
        moving = next(
            (
                arm
                for arm in ARMS
                if set(names) == {name for name in NAMES if name.startswith(arm + "_")}
            ),
            None,
        )
        if moving is None:
            raise ValueError(
                f"Single-arm goal must contain all six joints of one arm; received={list(names)}, expected={list(NAMES)}"
            )
        holding = "right" if moving == "left" else "left"
        sample = self.states.get(holding)
        if sample is None:
            raise ValueError(f"{holding}: no measured position for coordinated hold")
        state, stationary, _ = sample
        age = (
            self.get_clock().now().nanoseconds - nanoseconds(state.header.stamp)
        ) / 1e9
        held_names = [name for name in NAMES if name.startswith(holding + "_")]
        if not stationary or not 0 <= age <= 0.2 or set(state.name) != set(held_names):
            raise ValueError(
                f"arm={holding}: hold state invalid; stationary={stationary}, age={age:.6f} s (required 0..0.2 s), received joints={list(state.name)}, expected={held_names}"
            )
        positions = dict(zip(state.name, state.position))
        if len(positions) != 6 or not all(math.isfinite(x) for x in positions.values()):
            raise ValueError(
                f"arm={holding}: invalid hold positions (require six finite radians); measured={positions}"
            )
        result = copy.deepcopy(goal)
        result.trajectory.joint_names.extend(held_names)
        for point in result.trajectory.points:
            for field in ("positions", "velocities", "accelerations"):
                values = getattr(point, field)
                if field == "positions" or values:
                    if len(values) != 6:
                        raise ValueError(
                            f"arm={moving}: single-arm {field} must have six values; received={len(values)}"
                        )
                    values.extend(
                        [positions[name] for name in held_names]
                        if field == "positions"
                        else [0.0] * 6
                    )
        return result

    def update_health(self, arm, message):
        self.health[arm] = (
            time.monotonic(),
            bool(message.status)
            and all(
                (
                    int.from_bytes(status.level, "little")
                    if isinstance(status.level, bytes)
                    else status.level
                )
                < 2
                for status in message.status
            ),
        )

    def accept(self, goal):
        try:
            if self.stopping:
                raise ValueError("Coordinator is shutting down")
            if self.busy or self.fault:
                raise ValueError(self.fault or "A coordinated goal is already active")
            unavailable = [
                child_context(index)
                for index, client in enumerate(self.arm_clients)
                if not client.server_is_ready()
            ]
            if unavailable:
                raise ValueError(
                    f"Arm action servers unavailable: {'; '.join(unavailable)}"
                )
            for arm in ARMS:
                stamp, healthy = self.health.get(arm, (0, False))
                if (
                    not healthy
                    or time.monotonic() - stamp > self.config["diagnostics_timeout_s"]
                ):
                    raise ValueError(
                        f"arm={arm}: diagnostics missing, stale, or faulted; healthy={healthy}, age={time.monotonic() - stamp:.6f} s, timeout={self.config['diagnostics_timeout_s']} s; inspect /{arm}/iimt_ros2_driver/diagnostics"
                    )
            validate(self.normalized(goal).trajectory)
            if any(
                getattr(goal, field, [])
                for field in ("component_path_tolerance", "component_goal_tolerance")
            ):
                raise ValueError("Component tolerances are unsupported")
            if goal.multi_dof_trajectory.points:
                raise ValueError("Multi-DOF trajectories are unsupported")
            for field in ("path_tolerance", "goal_tolerance"):
                names = [item.name for item in getattr(goal, field)]
                if len(set(names)) != len(names) or any(
                    name not in NAMES for name in names
                ):
                    raise ValueError(
                        f"Invalid {field} joint names: received={names}; expected unique names from {list(NAMES)}"
                    )
            scheduled_start(
                nanoseconds(goal.trajectory.header.stamp),
                self.get_clock().now().nanoseconds,
                self.config["dispatch_lead_time_s"],
                self.required,
            )
        except ValueError as error:
            self.get_logger().error(f"Reject coordinated trajectory: {error}")
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    async def tick(self):
        future = Future()
        timer = self.create_timer(
            0.01, lambda: self.wake(future), callback_group=self.group
        )
        try:
            await future
        finally:
            self.destroy_timer(timer)

    @staticmethod
    def wake(future):
        if not future.done():
            future.set_result(None)

    def late_cancel(self, future, index=None):
        """An acceptance arriving after cleanup must never leave an orphan goal."""
        try:
            handle = future.result()
            if handle.accepted:
                handle.cancel_goal_async()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(
                f"Late child acceptance/cancellation failed ({child_context(index) if index is not None else 'arm unknown'}): {type(error).__name__}: {error}"
            )

    async def stop_children(self, sends, handles, results):
        deadline = time.monotonic() + self.config["cancel_timeout_s"]
        canceled = set()
        while time.monotonic() < deadline:
            self.collect_handles(sends, handles, results)
            for index, handle in handles.items():
                if index not in canceled and not results[index].done():
                    child_call(index, "cancel request", handle.cancel_goal_async)
                    canceled.add(index)
            if all(send.done() for send in sends) and all(
                result.done() for result in results.values()
            ):
                # Terminal results, not just cancellation acknowledgement, settle ownership.
                for index, result in results.items():
                    child_call(index, "stop result retrieval", result.result)
                return
            await self.tick()
        pending = [
            child_context(index)
            for index, send in enumerate(sends)
            if not send.done() or (index in results and not results[index].done())
        ]
        self.fault = (
            f"Child stop could not be confirmed after {self.config['cancel_timeout_s']} s; "
            f"pending=[{'; '.join(pending)}]; restart only after physical stop verification"
        )
        self.get_logger().error(self.fault)
        for index, send in enumerate(sends):
            if not send.done():
                send.add_done_callback(
                    lambda future, index=index: self.late_cancel(future, index)
                )

    @staticmethod
    def collect_handles(sends, handles, results):
        for index, send in enumerate(sends):
            if send.done() and index not in handles:
                handle = child_call(index, "goal acceptance", send.result)
                if handle.accepted:
                    handles[index] = handle
                    results[index] = child_call(
                        index, "result request", handle.get_result_async
                    )

    async def execute(self, parent):
        sends, handles, results = [], {}, {}
        outcome = FollowJointTrajectory.Result()
        try:
            if self.stopping:
                raise RuntimeError("Coordinator is shutting down")
            goal = self.normalized(parent.request)
            start = scheduled_start(
                nanoseconds(goal.trajectory.header.stamp),
                self.get_clock().now().nanoseconds,
                self.config["dispatch_lead_time_s"],
                self.required,
            )
            for index, (arm, client) in enumerate(zip(ARMS, self.arm_clients)):
                child = split_goal(goal, arm)
                (
                    child.trajectory.header.stamp.sec,
                    child.trajectory.header.stamp.nanosec,
                ) = divmod(start, 10**9)
                sends.append(
                    child_call(index, "goal dispatch", client.send_goal_async, child)
                )
            prefire_window = (
                start - self.get_clock().now().nanoseconds
            ) / 1e9 - self.required
            accepted_by = time.monotonic() + min(
                self.config["acceptance_timeout_s"], max(0.0, prefire_window)
            )
            duration = nanoseconds(goal.trajectory.points[-1].time_from_start) / 1e9
            tolerance = nanoseconds(goal.goal_time_tolerance) / 1e9
            finish_by = time.monotonic() + max(
                0, (start - self.get_clock().now().nanoseconds) / 1e9
            )
            finish_by += duration + max(tolerance, self.config["completion_margin_s"])
            while True:
                if self.stopping:
                    raise RuntimeError("Coordinator shutdown: canceling both arms")
                if len(handles) < 2 and time.monotonic() > accepted_by:
                    pending = [
                        child_context(i) for i in range(len(sends)) if i not in handles
                    ]
                    raise RuntimeError(
                        f"Child acceptance missed the pre-dispatch deadline; pending=[{'; '.join(pending)}], "
                        f"nominal_start_ns={start}, dispatch_required_s={self.required}"
                    )
                self.collect_handles(sends, handles, results)
                if parent.is_cancel_requested:
                    raise RuntimeError("Coordinated goal canceled")
                for index, send in enumerate(sends):
                    if (
                        send.done()
                        and not child_call(
                            index, "goal acceptance", send.result
                        ).accepted
                    ):
                        raise RuntimeError(
                            f"Trajectory rejected by {child_context(index)}; "
                            "ROS goal rejection has no error payload: inspect that driver's rejection log"
                        )
                for index, result in results.items():
                    if result.done():
                        response = child_call(
                            index, "execution result retrieval", result.result
                        )
                        if (
                            response.status != GoalStatus.STATUS_SUCCEEDED
                            or response.result.error_code != 0
                        ):
                            raise RuntimeError(
                                f"Child failed: {child_result(index, response)}"
                            )
                if len(results) == 2 and all(
                    result.done() for result in results.values()
                ):
                    parent.succeed()
                    outcome.error_code = outcome.SUCCESSFUL
                    return outcome
                if time.monotonic() > finish_by:
                    pending = [
                        child_context(i)
                        for i, result in results.items()
                        if not result.done()
                    ]
                    raise RuntimeError(
                        f"Coordinated completion deadline exceeded; pending=[{'; '.join(pending)}], "
                        f"trajectory_duration_s={duration}, goal_time_tolerance_s={tolerance}, "
                        f"completion_margin_s={self.config['completion_margin_s']}"
                    )
                await self.tick()
        except Exception as error:  # noqa: BLE001
            try:
                await self.stop_children(sends, handles, results)
            except Exception as stop_error:  # noqa: BLE001
                self.fault = f"Uncertain child stop: {stop_error}"
                for index, send in enumerate(sends):
                    send.add_done_callback(
                        lambda future, index=index: self.late_cancel(future, index)
                    )
            outcome.error_code = outcome.PATH_TOLERANCE_VIOLATED
            outcome.error_string = (
                f"{error}; {self.fault}" if self.fault else str(error)
            )
            outcome.error_string = append_child_failures(outcome.error_string, results)
            self.get_logger().error(outcome.error_string)
            if parent.is_cancel_requested:
                parent.canceled()
            else:
                parent.abort()
            return outcome
        finally:
            self.busy = False


def main(args=None):
    # Keep ROS alive during signal handling so child cancellations can settle.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Coordinator()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    previous_handlers = {}

    def request_stop(_signum, _frame):
        node.stopping = True

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, request_stop)
    try:
        while rclpy.ok() and not node.stopping:
            executor.spin_once(timeout_sec=0.05)
    finally:
        node.stopping = True
        deadline = time.monotonic() + node.config["cancel_timeout_s"] + 0.2
        while rclpy.ok() and node.busy and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        if node.busy or node.fault:
            node.get_logger().error(
                "Shutdown cannot confirm both arms stopped; verify physical stop independently"
            )
        executor.shutdown(timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
