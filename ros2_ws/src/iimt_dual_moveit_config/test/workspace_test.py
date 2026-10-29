"""World geometry, correct relative transform and filled boundary regressions."""

import importlib.util
import math
import unittest
from pathlib import Path

from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix

spec = importlib.util.spec_from_file_location(
    "workspace_scene",
    Path(__file__).resolve().parents[1] / "scripts/workspace_scene.py",
)
scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene)


class WorkspaceTest(unittest.TestCase):
    def test_relative_right_base_rotation(self):
        bounds = scene.workspace_bounds(
            [1.0, 2.0, 3.0], [0.0, 0.0, math.pi / 2], [2.0, 0.0, 1.0], [0.4] * 4
        )
        for actual, expected in zip(bounds, [0.6, 1.6, 1.4, 4.4, -7.0, 14.0]):
            self.assertAlmostEqual(actual, expected)

    def test_robot_urdf_defaults(self):
        b = scene.workspace_bounds(
            [-0.0008297, -0.538, 0.0818739],
            [0.0, 0.0, -0.6981317008],
            [-0.345440489323, 0.411679944176, -0.082],
            [0.5] * 4,
        )
        for actual, expected in zip(
            b[:4], [-0.5008297, -1.038, 0.4991703, 0.4994100000]
        ):
            self.assertAlmostEqual(actual, expected, places=5)
        self.assertAlmostEqual(b[4] + scene.WALL_DEPTH, -0.0001261, places=8)

    def test_independent_margins(self):
        bounds = scene.workspace_bounds(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 2.0, 0.0],
            [0.1, 0.2, 0.3, 0.4],
        )
        for actual, expected in zip(bounds[:4], [-0.1, -0.2, 1.3, 2.4]):
            self.assertAlmostEqual(actual, expected)

    def test_solids_fill_outside_and_leave_inside_clear(self):
        bounds = [-0.4, -0.4, 0.4, 1.4, -10.0, 10.0]
        walls = scene.boundary_objects(bounds)
        self.assertEqual(
            {o.id for o in walls},
            {f"iimt_workspace_{v}" for v in ("x_min", "x_max", "y_min", "y_max")},
        )

        def occupied(point):
            return any(
                all(
                    abs(value - center) <= size / 2
                    for value, center, size in zip(
                        point,
                        (
                            o.primitive_poses[0].position.x,
                            o.primitive_poses[0].position.y,
                            o.primitive_poses[0].position.z,
                        ),
                        o.primitives[0].dimensions,
                    )
                )
                for o in walls
            )

        for point in (
            (-1.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 2.0, 0.0),
            (1.0, 2.0, 2.0),
        ):
            self.assertTrue(occupied(point))
        for z in (-100.0, -1.0, 0.0, 1.0, 100.0):
            self.assertFalse(occupied((0.0, 0.0, z)))  # No ceiling or floor.
        self.assertTrue(all(o.header.frame_id == "world" for o in walls))

    def test_floor_plane_and_invalid_height(self):
        floor = scene.floor_object(-0.15)
        self.assertEqual(floor.id, "iimt_workspace_floor")
        self.assertEqual(floor.header.frame_id, "world")
        self.assertEqual(list(floor.planes[0].coef), [0.0, 0.0, 1.0, 0.15])
        self.assertEqual(floor.plane_poses[0].orientation.w, 1.0)
        self.assertEqual(floor.plane_poses[0].position.z, 0.0)
        for height in (float("nan"), float("inf"), -float("inf")):
            with self.assertRaises(ValueError):
                scene.floor_object(height)

    def test_floor_mount_exceptions_preserve_existing_matrix(self):
        matrix = AllowedCollisionMatrix()
        matrix.entry_names = ["left_base_link", "left_shoulder_link", "tool"]
        matrix.entry_values = [
            AllowedCollisionEntry(enabled=row)
            for row in (
                [False, True, False],
                [True, False, False],
                [False, False, False],
            )
        ]
        matrix.default_entry_names = ["fixture"]
        matrix.default_entry_values = [True]
        merged = scene.floor_mount_exceptions(matrix)
        self.assertEqual(
            matrix.entry_names, ["left_base_link", "left_shoulder_link", "tool"]
        )
        for i in range(3):
            self.assertEqual(
                merged.entry_values[i].enabled[:3], matrix.entry_values[i].enabled
            )
        floor = merged.entry_names.index(scene.FLOOR_ID)
        for index, name in enumerate(merged.entry_names):
            expected = name in scene.MOUNT_LINKS
            self.assertEqual(merged.entry_values[index].enabled[floor], expected)
            self.assertEqual(merged.entry_values[floor].enabled[index], expected)
        self.assertEqual(merged.default_entry_names, ["fixture", scene.FLOOR_ID])
        self.assertEqual(merged.default_entry_values, [True, False])
        self.assertEqual(scene.floor_mount_exceptions(merged), merged)

    def test_old_floor_exclusions_are_reset(self):
        matrix = AllowedCollisionMatrix()
        matrix.entry_names = [scene.FLOOR_ID, "tool"]
        matrix.entry_values = [
            AllowedCollisionEntry(enabled=[True, True]) for _ in range(2)
        ]
        matrix.default_entry_names = [scene.FLOOR_ID]
        matrix.default_entry_values = [True]
        merged = scene.floor_mount_exceptions(matrix)
        self.assertFalse(merged.entry_values[0].enabled[1])
        self.assertEqual(merged.default_entry_values, [False])

    def test_invalid_bounds(self):
        for margin in (0.0, -1.0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                scene.workspace_bounds(
                    [0.0] * 3, [0.0] * 3, [0.0] * 3, [margin, 0.5, 0.5, 0.5]
                )
        with self.assertRaises(ValueError):
            scene.workspace_bounds([0.0] * 2, [0.0] * 3, [0.0] * 3, [0.4] * 4)


if __name__ == "__main__":
    unittest.main()
