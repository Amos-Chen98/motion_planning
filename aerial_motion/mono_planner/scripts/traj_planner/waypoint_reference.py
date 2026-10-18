"""Deterministic world/FLU root-tail references shared by scenario evaluations."""
from pathlib import Path

import numpy as np
import yaml
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_matrix


def build_samples(case_file, duration, rate):
    from traj_planner.nonholonomic_trajectory import (
        NonholonomicQuinticTrajectory, matrix_to_quaternion,
        rotation_matrix_from_tangent, rotation_about_x,
    )
    data = yaml.safe_load(Path(case_file).read_text())
    if data.get("frame_id") != "world" or len(data.get("waypoints", [])) != 4:
        raise ValueError("Expected four world-frame waypoints")
    anchor = [0.53, 0.0, 1.0, 0.0, 0.0, np.pi]  # FLU: forward along world -X.
    knots = np.array([anchor] + data["waypoints"] + [anchor], dtype=float)
    if knots.shape != (6, 6) or not np.isfinite(knots).all():
        raise ValueError("Nonfinite or malformed waypoints")
    trajectory = NonholonomicQuinticTrajectory(
        knots[:, :3], np.stack([euler_matrix(*w[3:])[:3, :3] for w in knots]),
        duration, 1e-6, 1e-3, 1e-6, 1e-5,
    )
    samples = []
    for index in range(int(np.ceil(duration * rate)) + 1):
        position, velocity, twist = trajectory.sample(min(index / rate, duration))
        rotation = rotation_matrix_from_tangent(velocity, 1e-6, 1e-3) @ rotation_about_x(twist)
        quaternion = matrix_to_quaternion(rotation)
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quaternion
        samples.append(msg)
    return samples

