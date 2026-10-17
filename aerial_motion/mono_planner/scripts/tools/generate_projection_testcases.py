#!/usr/bin/env python3
"""Generate clockwise round trips for the nonholonomic waypoint planner.

Right turns mean decreasing yaw when viewed from +Z. The continuous check
matches NonholonomicQuinticTrajectory with the configured root forward axis
at both endpoints, distance-proportional timing, and zero knot acceleration.
It allows zero yaw rate at knots but rejects left turns between them.
"""

import math
import random

import numpy as np
from numpy.polynomial import Polynomial

from generate_waypoint_testcases import (
    MONO_PLANNER_DIR,
    build_candidate_waypoints,
    build_segment_directions,
    clean_output_directory,
    forward_axis_from_pitch_yaw,
    load_config,
    output_directory,
    sample_waypoint_positions,
    validate_generated_case,
    validate_segment_directions,
    wrap_angle,
    write_case_file,
)

CONFIG_FILE = MONO_PLANNER_DIR / "config/waypoint_testcases/projection_robustness_sim.yaml"

def validate_right_turn_config(config):
    if config.waypoint_count < 2:
        raise ValueError("Right-turn round trips require at least two waypoints.")
    if math.hypot(*config.root_forward_axis[:2]) <= config.segment_epsilon:
        raise ValueError("Right turns require a nonzero horizontal root_forward_axis.")


def order_clockwise(positions, config):
    """Sort around the centroid, then start immediately after the root."""
    knots = [config.root_position, *positions]
    center_x = sum(position[0] for position in knots) / len(knots)
    center_y = sum(position[1] for position in knots) / len(knots)
    indices = sorted(
        range(len(knots)),
        key=lambda index: math.atan2(knots[index][1] - center_y, knots[index][0] - center_x),
        reverse=True,
    )
    root_index = indices.index(0)
    ordered_indices = indices[root_index + 1:] + indices[:root_index]
    return [knots[index] for index in ordered_indices]


def validate_right_turn_directions(segment_directions, config):
    violation = validate_segment_directions(segment_directions, config)
    if violation is not None:
        raise ValueError(violation)

    directions = [config.root_forward_axis, *segment_directions, config.root_forward_axis]
    headings = []
    for direction in directions:
        if math.hypot(*direction[:2]) <= config.segment_epsilon:
            raise ValueError("Right-turn paths cannot contain vertical segments.")
        headings.append(math.atan2(direction[1], direction[0]))

    turns = [wrap_angle(end - start) for start, end in zip(headings[:-1], headings[1:])]
    for index, turn in enumerate(turns):
        if not (-math.pi + config.angle_tolerance < turn < -config.angle_tolerance):
            raise ValueError(f"Direction transition {index} is not a strict right turn.")
    if not math.isclose(sum(turns), -2.0 * math.pi, rel_tol=0.0,
                        abs_tol=len(turns) * config.angle_tolerance):
        raise ValueError("The path must make exactly one clockwise round trip.")


def validate_continuous_right_turns(waypoints, config):
    """Check the yaw-rate sign over each complete quintic segment.

    Use normalized segment time u in [0, 1]. Distance-proportional timing
    makes endpoint derivatives equal to chord length times the forward axis,
    independent of total trajectory time. This is the position construction
    in NonholonomicQuinticTrajectory.build_position_segments, without ROS.
    """
    positions = np.asarray([config.root_position, *(waypoint[:3] for waypoint in waypoints),
                            config.root_position], dtype=float)
    root_axis = np.asarray(config.root_forward_axis, dtype=float)
    root_axis /= np.linalg.norm(root_axis)
    axes = np.asarray([root_axis, *(forward_axis_from_pitch_yaw(waypoint[4], waypoint[5])
                                   for waypoint in waypoints), root_axis])

    for index, displacement in enumerate(np.diff(positions, axis=0)):
        distance = np.linalg.norm(displacement)
        # Scale spatial coordinates by chord length for stable sign checks.
        chord = displacement / distance
        start_velocity, end_velocity = axes[index:index + 2]
        coefficients = np.vstack((
            np.zeros(3),
            start_velocity,
            np.zeros(3),
            10.0 * chord - 6.0 * start_velocity - 4.0 * end_velocity,
            -15.0 * chord + 8.0 * start_velocity + 7.0 * end_velocity,
            6.0 * chord - 3.0 * start_velocity - 3.0 * end_velocity,
        ))
        velocity_x = Polynomial(coefficients[:, 0]).deriv()
        velocity_y = Polynomial(coefficients[:, 1]).deriv()
        # yaw_rate has the sign of vx * ay - vy * ax.
        turn_numerator = velocity_x * velocity_y.deriv() - velocity_y * velocity_x.deriv()
        # Zero knot acceleration supplies the u*(1-u) factor. Removing it
        # lets us require a strictly negative sign throughout the interior.
        interior_turn, remainder = divmod(turn_numerator, Polynomial([0.0, 1.0, -1.0]))
        if np.max(np.abs(remainder.coef)) > config.dot_epsilon:
            raise ValueError(f"Segment {index} has an unstable curvature polynomial.")

        critical_times = [0.0, 1.0]
        for root in interior_turn.deriv().roots():
            if abs(root.imag) <= config.angle_tolerance and 0.0 < root.real < 1.0:
                critical_times.append(float(root.real))
        if max(interior_turn(time) for time in critical_times) >= -config.dot_epsilon:
            raise ValueError(f"Segment {index} does not turn right throughout its interior.")


def validate_right_turn_case(waypoints, config):
    validate_generated_case(waypoints, config)
    positions = [tuple(waypoint[:3]) for waypoint in waypoints]
    validate_right_turn_directions(build_segment_directions(positions, config), config)
    validate_continuous_right_turns(waypoints, config)


def generate_waypoint_case(rng, config):
    for _attempt in range(config.max_attempts_per_case):
        positions = order_clockwise(sample_waypoint_positions(rng, config), config)
        try:
            validate_right_turn_directions(build_segment_directions(positions, config), config)
        except ValueError:
            continue
        pitches = [rng.uniform(*config.pitch_range) for _ in positions]
        waypoints = build_candidate_waypoints(positions, pitches, config)
        if waypoints is None:
            continue
        try:
            # Recheck the rounded values that will actually be written.
            validate_right_turn_case(waypoints, config)
        except ValueError:
            continue
        return waypoints

    raise RuntimeError(
        f"Failed to generate a continuously right-turning {config.waypoint_count}-waypoint case "
        f"within {config.max_attempts_per_case} attempts."
    )


def main():
    config = load_config(CONFIG_FILE)
    validate_right_turn_config(config)
    rng = random.Random(config.random_seed)
    # Finish generation before replacing any existing output files.
    cases = [generate_waypoint_case(rng, config) for _ in range(config.num_cases)]
    target_directory = output_directory(config)
    target_directory.mkdir(parents=True, exist_ok=True)
    removed_count = clean_output_directory(target_directory, config)
    filename_width = max(4, len(str(config.num_cases)))
    for case_index, waypoints in enumerate(cases, start=1):
        file_name = f"{config.file_prefix}{case_index:0{filename_width}d}.yaml"
        write_case_file(target_directory / file_name, waypoints, config)
    print(
        f"Generated {config.num_cases} right-turn waypoint testcases in {target_directory} "
        f"(removed {removed_count} existing {config.file_prefix}*.yaml files)."
    )


if __name__ == "__main__":
    main()
