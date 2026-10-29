"""Pure validation and name mapping; timestamps and CSV points are not retimed."""

import copy
import math

JOINTS = (
    "shoulder_joint",
    "upper_arm_joint",
    "forearm_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
ARMS = ("left", "right")
NAMES = tuple(f"{arm}_{joint}" for arm in ARMS for joint in JOINTS)


def nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def validate(trajectory):
    names = trajectory.joint_names
    if len(names) != 12 or set(names) != set(NAMES):
        raise ValueError(
            f"Exactly the twelve distinct left_/right_ joint names are required; received={list(names)}, expected={list(NAMES)}"
        )
    if not trajectory.points:
        raise ValueError("Trajectory has no points")
    if not 0 <= trajectory.header.stamp.nanosec < 1_000_000_000:
        raise ValueError(
            f"Invalid header timestamp: sec={trajectory.header.stamp.sec}, nanosec={trajectory.header.stamp.nanosec}; nanosec must be in [0, 1000000000)"
        )
    derivative_shape = tuple(
        bool(getattr(trajectory.points[0], field))
        for field in ("velocities", "accelerations")
    )
    previous = -1
    for point_index, point in enumerate(trajectory.points):
        duration = point.time_from_start
        now = nanoseconds(duration)
        if not 0 <= duration.nanosec < 1_000_000_000 or now <= previous:
            raise ValueError(
                f"Point {point_index}: times must be nonnegative and strictly increasing; time_from_start_ns={now}, previous_ns={previous}, sec={duration.sec}, nanosec={duration.nanosec}; nanosec must be in [0, 1000000000)"
            )
        previous = now
        for field in ("positions", "velocities", "accelerations"):
            values = getattr(point, field)
            if (field == "positions" or values) and len(values) != 12:
                raise ValueError(
                    f"Point {point_index}, time_from_start_ns={now}: {field} must contain twelve values when supplied; received_size={len(values)}"
                )
            for joint_index, value in enumerate(values):
                if not math.isfinite(value):
                    raise ValueError(
                        f"Point {point_index}, time_from_start_ns={now}, "
                        f"joint={names[joint_index]}: {field} has nonfinite value={value}"
                    )
        if (
            tuple(
                bool(getattr(point, field)) for field in ("velocities", "accelerations")
            )
            != derivative_shape
        ):
            raise ValueError(
                f"Point {point_index}, time_from_start_ns={now}: derivative presence must be consistent; expected velocities/accelerations={derivative_shape}, received={(bool(point.velocities), bool(point.accelerations))}"
            )
        if point.accelerations and not point.velocities:
            raise ValueError(
                f"Point {point_index}, time_from_start_ns={now}: accelerations require velocities; accelerations_size={len(point.accelerations)}, velocities_size={len(point.velocities)}"
            )
        if point.effort:
            raise ValueError(
                f"Point {point_index}, time_from_start_ns={now}: effort commands are unsupported; received_size={len(point.effort)}"
            )


def split_goal(goal, arm):
    result = copy.deepcopy(goal)
    names = [f"{arm}_{joint}" for joint in JOINTS]
    indices = [goal.trajectory.joint_names.index(name) for name in names]
    result.trajectory.joint_names = names
    for point in result.trajectory.points:
        for field in ("positions", "velocities", "accelerations"):
            values = getattr(point, field)
            if values:
                setattr(point, field, [values[index] for index in indices])
    for field in ("path_tolerance", "goal_tolerance"):
        setattr(
            result,
            field,
            [
                copy.deepcopy(tolerance)
                for tolerance in getattr(goal, field)
                if tolerance.name in names
            ],
        )
    return result


def scheduled_start(requested_ns, now_ns, lead_s, required_s):
    start = requested_ns or now_ns + round(lead_s * 1e9)
    if start - now_ns < round(required_s * 1e9):
        raise ValueError(
            f"Start stamp leaves insufficient dispatch/pre-roll margin; requested_ns={requested_ns}, scheduled_ns={start}, now_ns={now_ns}, remaining_s={(start - now_ns) / 1e9:.9f}, required_s={required_s:.9f}, lead_s={lead_s:.9f}"
        )
    return start
