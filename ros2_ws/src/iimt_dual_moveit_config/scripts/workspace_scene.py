#!/usr/bin/env python3
"""Install solid side boundaries in MoveIt's world planning scene."""

import copy
import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from shape_msgs.msg import Plane, SolidPrimitive

WALL_DEPTH = 10.0  # Far beyond the fixed CI05's reachable envelope; not a ceiling.


def workspace_bounds(left_xyz, left_rpy, right_xyz, margins):
    """World X/Y bounds; right_xyz is expressed in the left base frame."""
    if not all(len(v) == 3 for v in (left_xyz, left_rpy, right_xyz)):
        raise ValueError("Base translations and rotations require three values")
    if not all(math.isfinite(v) for a in (left_xyz, left_rpy, right_xyz) for v in a):
        raise ValueError("Base transforms must be finite")
    if len(margins) != 4 or any(not math.isfinite(v) or v <= 0 for v in margins):
        raise ValueError(
            "Workspace margins [x_min, y_min, x_max, y_max] must be finite and positive"
        )
    r, p, y = left_rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    right = [
        left_xyz[i] + sum(rotation[i][j] * right_xyz[j] for j in range(3))
        for i in range(3)
    ]
    return (
        [min(left_xyz[i], right[i]) - margins[i] for i in range(2)]
        + [max(left_xyz[i], right[i]) + margins[i + 2] for i in range(2)]
        + [
            min(left_xyz[2], right[2]) - WALL_DEPTH,
            max(left_xyz[2], right[2]) + WALL_DEPTH,
        ]
    )


def boundary_objects(bounds):
    """Four filled outside slabs. No horizontal floor or ceiling is installed."""
    xmin, ymin, xmax, ymax, zmin, zmax = bounds
    depth = WALL_DEPTH
    xmid, ymid, zmid = (xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2
    specifications = (
        (
            "x_min",
            (xmin - depth / 2, ymid, zmid),
            (depth, ymax - ymin + 2 * depth, zmax - zmin),
        ),
        (
            "x_max",
            (xmax + depth / 2, ymid, zmid),
            (depth, ymax - ymin + 2 * depth, zmax - zmin),
        ),
        (
            "y_min",
            (xmid, ymin - depth / 2, zmid),
            (xmax - xmin + 2 * depth, depth, zmax - zmin),
        ),
        (
            "y_max",
            (xmid, ymax + depth / 2, zmid),
            (xmax - xmin + 2 * depth, depth, zmax - zmin),
        ),
    )
    objects = []
    for suffix, center, size in specifications:
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = f"iimt_workspace_{suffix}"
        obj.operation = CollisionObject.ADD
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x, pose.position.y, pose.position.z = center
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        objects.append(obj)
    return objects


FLOOR_ID = "iimt_workspace_floor"
MOUNT_LINKS = ("left_base_link", "right_base_link")


def floor_object(height):
    """Horizontal infinite surface, not a filled forbidden halfspace."""
    if not math.isfinite(height):
        raise ValueError("floor_z must be finite")
    obj = CollisionObject()
    obj.header.frame_id = "world"
    obj.id = FLOOR_ID
    obj.operation = CollisionObject.ADD
    obj.planes = [Plane(coef=[0.0, 0.0, 1.0, -height])]
    pose = Pose()
    pose.orientation.w = 1.0
    obj.plane_poses = [pose]
    return obj


def floor_mount_exceptions(existing):
    """Preserve the existing ACM, allowing only the two fixed mounts vs floor."""
    matrix = copy.deepcopy(existing)
    names = matrix.entry_names
    if len(matrix.entry_values) != len(names) or any(
        len(row.enabled) != len(names) for row in matrix.entry_values
    ):
        raise ValueError("MoveIt returned a malformed allowed collision matrix")
    for name in (*MOUNT_LINKS, FLOOR_ID):
        if name not in names:
            names.append(name)
            for row in matrix.entry_values:
                row.enabled.append(False)
            matrix.entry_values.append(
                AllowedCollisionEntry(enabled=[False] * len(names))
            )
    floor_index = names.index(FLOOR_ID)
    for index, name in enumerate(names):
        allowed = name in MOUNT_LINKS
        matrix.entry_values[floor_index].enabled[index] = allowed
        matrix.entry_values[index].enabled[floor_index] = allowed
    # Explicitly keep floor collision checking on for future/unlisted bodies too.
    if FLOOR_ID in matrix.default_entry_names:
        matrix.default_entry_values[matrix.default_entry_names.index(FLOOR_ID)] = False
    else:
        matrix.default_entry_names.append(FLOOR_ID)
        matrix.default_entry_values.append(False)
    return matrix


class WorkspaceScene(Node):
    def __init__(self):
        super().__init__("iimt_workspace")
        readonly = ParameterDescriptor(read_only=True)
        margin = self.declare_parameter("workspace_margin", 0.5, readonly).value
        if not math.isfinite(margin) or margin <= 0:
            raise ValueError("workspace_margin must be finite and positive")
        sides = ("x_min", "y_min", "x_max", "y_max")
        defaults = {
            "left_base_xyz": [0.0, 0.0, 0.0],
            "left_base_rpy": [0.0, 0.0, 0.0],
            "right_base_xyz": [0.0, 1.2, 0.0],
            **{f"workspace_{side}_margin": margin for side in sides},
        }
        values = {
            name: self.declare_parameter(name, value, readonly).value
            for name, value in defaults.items()
        }
        bounds = workspace_bounds(
            values["left_base_xyz"],
            values["left_base_rpy"],
            values["right_base_xyz"],
            [values[f"workspace_{side}_margin"] for side in sides],
        )
        self.declare_parameter("bounds_xyz", bounds, readonly)
        self.scene = PlanningScene(is_diff=True)
        self.scene.robot_state.is_diff = True
        floor_z = self.declare_parameter(
            "floor_z", bounds[4] + WALL_DEPTH, readonly
        ).value
        self.scene.world.collision_objects = boundary_objects(bounds) + [
            floor_object(floor_z)
        ]
        self.reader = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.reading = True
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.pending = None
        self.timer = self.create_timer(1.0, self.apply)
        self.get_logger().info(
            f"World workspace: x=[{bounds[0]:.6f}, {bounds[2]:.6f}], y=[{bounds[1]:.6f}, {bounds[3]:.6f}]; floor z={floor_z:.6f}, no ceiling"
        )

    def apply(self):
        if self.pending is not None:
            if not self.pending.done():
                return
            try:
                response = self.pending.result()
                if self.reading:
                    self.scene.allowed_collision_matrix = floor_mount_exceptions(
                        response.scene.allowed_collision_matrix
                    )
                    self.reading = False
                elif response.success:
                    self.get_logger().info(
                        "Workspace boundaries and floor applied; fixed mount contacts allowed"
                    )
                    self.timer.cancel()
                    self.pending = None
                    return
                else:
                    self.get_logger().error("MoveIt rejected workspace scene; retrying")
                    self.reading = True
            except Exception as error:  # noqa: BLE001 - ROS future error boundary
                self.get_logger().error(f"Workspace scene update failed: {error}")
                self.reading = True
            self.pending = None
        if self.reading:
            if self.reader.service_is_ready():
                request = GetPlanningScene.Request()
                request.components.components = (
                    PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                )
                self.pending = self.reader.call_async(request)
        elif self.client.service_is_ready():
            self.pending = self.client.call_async(
                ApplyPlanningScene.Request(scene=self.scene)
            )


def main():
    rclpy.init()
    node = WorkspaceScene()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
