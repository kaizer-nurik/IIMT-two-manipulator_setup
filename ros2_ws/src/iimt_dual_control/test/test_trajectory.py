import unittest
from types import SimpleNamespace as Obj

from iimt_dual_control.trajectory import NAMES, scheduled_start, split_goal, validate


def goal():
    point = Obj(
        positions=list(range(12)),
        velocities=[],
        accelerations=[],
        effort=[],
        time_from_start=Obj(sec=0, nanosec=0),
    )
    return Obj(
        trajectory=Obj(
            joint_names=list(reversed(NAMES)),
            points=[point],
            header=Obj(stamp=Obj(sec=10, nanosec=42)),
        ),
        path_tolerance=[],
        goal_tolerance=[],
    )


class TrajectoryTests(unittest.TestCase):
    def test_mapping_preserves_stamp_and_positions_only(self):
        source = goal()
        validate(source.trajectory)
        for arm in ("left", "right"):
            result = split_goal(source, arm)
            self.assertEqual(result.trajectory.header.stamp.nanosec, 42)
            self.assertEqual(result.trajectory.points[0].velocities, [])
            self.assertEqual(
                result.trajectory.points[0].positions,
                [
                    source.trajectory.joint_names.index(name)
                    for name in result.trajectory.joint_names
                ],
            )
        self.assertEqual(len(source.trajectory.joint_names), 12)

    def test_invalid_names_and_arrays(self):
        for change in (
            lambda g: g.trajectory.joint_names.__setitem__(0, NAMES[0]),
            lambda g: g.trajectory.points[0].positions.pop(),
            lambda g: g.trajectory.points[0].positions.__setitem__(0, float("nan")),
        ):
            source = goal()
            change(source)
            with self.assertRaises(ValueError):
                validate(source.trajectory)

    def test_validation_details_identify_point_field_and_joint(self):
        source = goal()
        source.trajectory.points[0].positions[0] = float("nan")
        with self.assertRaisesRegex(
            ValueError,
            "Point 0.*time_from_start_ns=0.*joint=right_wrist_3_joint.*positions.*value=nan",
        ):
            validate(source.trajectory)
        source = goal()
        source.trajectory.points[0].positions.pop()
        with self.assertRaisesRegex(ValueError, "Point 0.*positions.*received_size=11"):
            validate(source.trajectory)
        source = goal()
        source.trajectory.joint_names[0] = "unknown_joint"
        with self.assertRaisesRegex(ValueError, "received=.*unknown_joint.*expected="):
            validate(source.trajectory)

    def test_invalid_header_timestamp_details(self):
        source = goal()
        source.trajectory.header.stamp.nanosec = 1000000000
        with self.assertRaisesRegex(ValueError, "sec=10, nanosec=1000000000"):
            validate(source.trajectory)

    def test_nonincreasing_time(self):
        source = goal()
        source.trajectory.points *= 2
        with self.assertRaisesRegex(
            ValueError, "Point 1.*time_from_start_ns=0, previous_ns=0"
        ):
            validate(source.trajectory)

    def test_start_preserves_future_and_rejects_insufficient_preroll(self):
        self.assertEqual(scheduled_start(10**10, 9 * 10**9, 1, 0.53), 10**10)
        self.assertEqual(scheduled_start(0, 9 * 10**9, 1, 0.53), 10**10)
        with self.assertRaisesRegex(
            ValueError, "remaining_s=0.100000000, required_s=0.530000000"
        ):
            scheduled_start(10**10, 99 * 10**8, 1, 0.53)


if __name__ == "__main__":
    unittest.main()
