"""No ROS graph or hardware: exercise coordinator failure cleanup and hold validation."""

import asyncio
import time
import unittest
from types import SimpleNamespace as Obj

try:
    from control_msgs.action import FollowJointTrajectory
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from iimt_dual_control.coordinator import Coordinator
    from iimt_dual_control.trajectory import NAMES
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectoryPoint
except ImportError:
    Coordinator = None


class Ready:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value


class Handle:
    accepted = True

    def __init__(self):
        self.canceled = False
        self.result_future = Obj(
            done=lambda: self.canceled,
            result=lambda: Obj(
                status=5, result=Obj(error_code=0, error_string="Canceled")
            ),
        )

    def cancel_goal_async(self):
        self.canceled = True

    def get_result_async(self):
        return self.result_future


@unittest.skipIf(Coordinator is None, "Requires ROS message Python packages")
class CoordinatorTests(unittest.IsolatedAsyncioTestCase):
    def fake(self):
        fake = Obj(config={"cancel_timeout_s": 0.03}, fault="", states={})
        fake.collect_handles = Coordinator.collect_handles
        fake.tick = lambda: asyncio.sleep(0.001)
        fake.get_logger = lambda: Obj(error=lambda _: None)
        fake.late_cancel = lambda future, index=None: Coordinator.late_cancel(
            fake, future, index
        )
        fake.get_clock = lambda: Obj(now=lambda: Obj(nanoseconds=10**10))
        return fake

    async def test_rejected_child_cancels_accepted_child_and_settles(self):
        fake = self.fake()
        handle = Handle()
        sends = [Ready(handle), Ready(Obj(accepted=False))]
        await Coordinator.stop_children(fake, sends, {}, {})
        self.assertTrue(handle.canceled)
        self.assertEqual(fake.fault, "")

    async def test_unresolved_acceptance_latches_and_cancels_late_handle(self):
        fake = self.fake()
        callbacks = []
        pending = Obj(done=lambda: False, add_done_callback=callbacks.append)
        await Coordinator.stop_children(fake, [pending], {}, {})
        self.assertIn("could not be confirmed", fake.fault)
        handle = Handle()
        callbacks[0](Ready(handle))
        self.assertTrue(handle.canceled)

    async def test_real_node_construction(self):
        import rclpy

        rclpy.init()
        node = None
        try:
            node = Coordinator()
            self.assertEqual(len(node.arm_clients), 2)
            from rclpy.parameter import Parameter

            result = node.set_parameters([Parameter("cancel_timeout_s", value=4.0)])[0]
            self.assertFalse(result.successful)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

    async def test_real_diagnostic_byte_levels(self):
        fake = self.fake()
        fake.health = {}
        for level, healthy in (
            (DiagnosticStatus.OK, True),
            (DiagnosticStatus.WARN, True),
            (DiagnosticStatus.ERROR, False),
        ):
            message = DiagnosticArray()
            status = DiagnosticStatus()
            status.level = level
            message.status = [status]
            Coordinator.update_health(fake, "left", message)
            self.assertEqual(fake.health["left"][1], healthy)

    def execution(self, handles):
        fake = self.fake()
        fake.config.update(
            dispatch_lead_time_s=0.08,
            dispatch_margin_s=0.04,
            acceptance_timeout_s=2.0,
            completion_margin_s=0.1,
        )
        fake.required = 0.04
        fake.busy = True
        fake.stopping = False
        fake.normalized = lambda goal: goal
        fake.stop_children = lambda sends, accepted, results: Coordinator.stop_children(
            fake, sends, accepted, results
        )
        fake.arm_clients = [
            Obj(send_goal_async=lambda _, h=handle: Ready(h)) for handle in handles
        ]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(NAMES)
        point = JointTrajectoryPoint()
        point.positions = [0.0] * 12
        point.time_from_start.nanosec = 10_000_000
        goal.trajectory.points = [point]
        parent = Obj(request=goal, is_cancel_requested=False, state="")
        parent.succeed = lambda: setattr(parent, "state", "succeeded")
        parent.abort = lambda: setattr(parent, "state", "aborted")
        parent.canceled = lambda: setattr(parent, "state", "canceled")
        return fake, parent

    async def test_parent_cancel_cancels_both_children(self):
        handles = [Handle(), Handle()]
        fake, parent = self.execution(handles)
        parent.is_cancel_requested = True
        await Coordinator.execute(fake, parent)
        self.assertEqual(parent.state, "canceled")
        self.assertTrue(all(handle.canceled for handle in handles))
        self.assertFalse(fake.busy)

    async def test_shutdown_cancels_owned_children(self):
        handles = [Handle(), Handle()]
        fake, parent = self.execution(handles)

        async def shutdown_tick():
            fake.stopping = True
            await asyncio.sleep(0.001)

        fake.tick = shutdown_tick
        await Coordinator.execute(fake, parent)
        self.assertEqual(parent.state, "aborted")
        self.assertTrue(all(handle.canceled for handle in handles))
        self.assertFalse(fake.busy)
        self.assertEqual(fake.fault, "")

    async def test_failed_result_cancels_peer(self):
        handles = [Handle(), Handle()]
        handles[0].result_future = Ready(
            Obj(status=6, result=Obj(error_code=-4, error_string="fault"))
        )
        fake, parent = self.execution(handles)
        outcome = await Coordinator.execute(fake, parent)
        self.assertEqual(parent.state, "aborted")
        self.assertTrue(handles[1].canceled)
        self.assertIn("arm=left", outcome.error_string)
        self.assertIn(
            "/left/iimt_group_controller/follow_joint_trajectory", outcome.error_string
        )
        self.assertIn("status=6, error_code=-4: fault", outcome.error_string)

    async def test_failed_sibling_stop_is_reported_without_duplicating_primary(self):
        handles = [Handle(), Handle()]
        handles[0].result_future = Ready(
            Obj(status=6, result=Obj(error_code=-4, error_string="left tracking fault"))
        )
        handles[1].result_future = Obj(
            done=lambda: handles[1].canceled,
            result=lambda: Obj(
                status=6,
                result=Obj(
                    error_code=-4,
                    error_string="SDK Stop/Clear failed; physical stop is NOT confirmed",
                ),
            ),
        )
        fake, parent = self.execution(handles)
        outcome = await Coordinator.execute(fake, parent)
        self.assertEqual(parent.state, "aborted")
        self.assertEqual(
            fake.fault, ""
        )  # Diagnostic change must not alter ownership policy.
        self.assertEqual(outcome.error_string.count("left tracking fault"), 1)
        self.assertIn("arm=left", outcome.error_string)
        self.assertIn("arm=right", outcome.error_string)
        self.assertIn("SDK Stop/Clear failed", outcome.error_string)
        self.assertTrue(handles[1].canceled)

    async def test_successful_sibling_cancel_is_not_reported_as_failure(self):
        handles = [Handle(), Handle()]
        handles[0].result_future = Ready(
            Obj(status=6, result=Obj(error_code=-4, error_string="left tracking fault"))
        )
        fake, parent = self.execution(handles)
        outcome = await Coordinator.execute(fake, parent)
        self.assertNotIn("Child cleanup result", outcome.error_string)
        self.assertEqual(outcome.error_string.count("left tracking fault"), 1)
        self.assertTrue(handles[1].canceled)

    async def test_right_child_details_survive_stop_cleanup_failure(self):
        handles = [Handle(), Handle()]
        details = (
            "joint=right_joint_2 desired=0.5 actual=0.7 error=0.2 tolerance=0.1 rad; "
            "SDK Stop/Clear failed; physical stop is NOT confirmed"
        )
        handles[1].result_future = Ready(
            Obj(status=6, result=Obj(error_code=-4, error_string=details))
        )
        fake, parent = self.execution(handles)

        async def failing_stop(*_):
            raise RuntimeError("cancel transport unavailable")

        fake.stop_children = failing_stop
        for client in fake.arm_clients:
            original = client.send_goal_async

            def send(goal, original=original):
                future = original(goal)
                future.add_done_callback = lambda _: None
                return future

            client.send_goal_async = send
        outcome = await Coordinator.execute(fake, parent)
        self.assertIn("arm=right", outcome.error_string)
        self.assertIn(details, outcome.error_string)
        self.assertIn("cancel transport unavailable", outcome.error_string)
        self.assertEqual(parent.state, "aborted")

    async def test_dispatch_transport_error_names_arm_without_joint_guess(self):
        fake, parent = self.execution([Handle(), Handle()])

        def failed_send(_):
            raise RuntimeError("transport disconnected")

        fake.arm_clients[1].send_goal_async = failed_send
        outcome = await Coordinator.execute(fake, parent)
        self.assertIn("arm=right", outcome.error_string)
        self.assertIn(
            "goal dispatch failed: RuntimeError: transport disconnected",
            outcome.error_string,
        )
        self.assertNotIn("desired=", outcome.error_string)
        self.assertEqual(parent.state, "aborted")

    async def test_child_rejection_cancels_peer(self):
        handles = [Handle(), Obj(accepted=False)]
        fake, parent = self.execution(handles)
        outcome = await Coordinator.execute(fake, parent)
        self.assertEqual(parent.state, "aborted")
        self.assertTrue(handles[0].canceled)
        self.assertIn("arm=right", outcome.error_string)
        self.assertIn("no error payload", outcome.error_string)
        self.assertNotIn("desired=", outcome.error_string)

    async def test_delayed_acceptance_canceled_before_prefire_and_latched(self):
        handles = [Handle(), Handle()]
        fake, parent = self.execution(handles)
        callbacks = []
        pending = Obj(done=lambda: False, add_done_callback=callbacks.append)
        fake.arm_clients[1] = Obj(send_goal_async=lambda _: pending)
        started = time.monotonic()
        await Coordinator.execute(fake, parent)
        self.assertLess(time.monotonic() - started, 0.2)
        self.assertEqual(parent.state, "aborted")
        self.assertTrue(handles[0].canceled)
        self.assertTrue(fake.fault)
        callbacks[0](Ready(handles[1]))
        self.assertTrue(handles[1].canceled)

    async def test_single_arm_holds_only_fresh_stationary_other_arm(self):
        fake = self.fake()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(NAMES[:6])
        point = JointTrajectoryPoint()
        point.positions = [0.0] * 6
        goal.trajectory.points = [point]
        state = JointState()
        state.name = list(NAMES[6:])
        state.position = [0.1] * 6
        state.header.stamp.sec = 10
        fake.states["right"] = (state, True, 10**10)
        expanded = Coordinator.normalized(fake, goal)
        self.assertEqual(
            list(expanded.trajectory.points[0].positions), [0.0] * 6 + [0.1] * 6
        )
        self.assertEqual(list(expanded.trajectory.points[0].velocities), [])
        fake.states["right"] = (state, False, 10**10)
        with self.assertRaises(ValueError):
            Coordinator.normalized(fake, goal)
        state.header.stamp.sec = 9
        fake.states["right"] = (state, True, 10**10)
        with self.assertRaises(ValueError):
            Coordinator.normalized(fake, goal)


if __name__ == "__main__":
    unittest.main()
