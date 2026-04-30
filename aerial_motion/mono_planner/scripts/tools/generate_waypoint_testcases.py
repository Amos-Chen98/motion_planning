#!/usr/bin/env python3

import itertools
import math
import random
from pathlib import Path

NUM_CASES = 100
RANDOM_SEED = 0
FRAME_ID = "world"
OUTPUT_DIR = Path("test_data")
FILE_PREFIX = "case_"
ROOT_POSITION = (0.0, 0.0, 1.0)
ROOT_YAW = math.pi
ROLL = 0.0
DIAMETER_RANGE = (1.0, 4.0)
PITCH_RANGE = (-0.2, 0.2)
CIRCLE_Z_RANGE = (0.8, 1.6)
CENTER_ANGLE_RANGE = (math.radians(114.0), math.radians(170.0))
MAX_ATTEMPTS_PER_CASE = 1000

WAYPOINT_COUNT = 4
ROUND_DIGITS = 9
SEGMENT_EPSILON = 1e-9
DOT_EPSILON = 1e-9
ANGLE_TOLERANCE = 1e-9
CIRCLE_TOLERANCE = 5e-6
HALF_SPACE_EPSILON = 1e-6
ROOT_XY_EXCLUSION_RADIUS = 0.5
MIN_WAYPOINT_ANGULAR_SEPARATION = math.radians(60.0)
MAX_YAW_STEP_DEGREES = 80.0
MAX_YAW_STEP = math.radians(MAX_YAW_STEP_DEGREES)
MAX_POSITION_SAMPLING_ATTEMPTS = 200

SCRIPT_DIR = Path(__file__).resolve().parent
MONO_PLANNER_DIR = SCRIPT_DIR.parent.parent
ROOT_FORWARD_AXIS = (-1.0, 0.0, 0.0)


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def dot(left, right):
    return sum(left_value * right_value for left_value, right_value in zip(left, right))


def subtract(left, right):
    return tuple(left_value - right_value for left_value, right_value in zip(left, right))


def norm(vector):
    return math.sqrt(dot(vector, vector))


def unit_vector(vector):
    vector_norm = norm(vector)
    if vector_norm <= SEGMENT_EPSILON:
        return None
    return tuple(component / vector_norm for component in vector)


def xy_distance(point, center):
    return math.hypot(point[0] - center[0], point[1] - center[1])


def angular_distance(angle_a, angle_b):
    return abs(wrap_angle(angle_a - angle_b))


def point_angle_on_circle(point, circle_center):
    return math.atan2(point[1] - circle_center[1], point[0] - circle_center[0])


def forward_axis_from_pitch_yaw(pitch, yaw):
    cosine_pitch = math.cos(pitch)
    return (
        math.cos(yaw) * cosine_pitch,
        math.sin(yaw) * cosine_pitch,
        -math.sin(pitch),
    )


def find_closed_loop_yaw_step_violation(waypoint_yaws):
    yaw_labels = ["ROOT"]
    yaw_sequence = [ROOT_YAW]

    for waypoint_index, yaw in enumerate(waypoint_yaws):
        yaw_labels.append(f"waypoint_{waypoint_index}")
        yaw_sequence.append(yaw)

    yaw_labels.append("ROOT")
    yaw_sequence.append(ROOT_YAW)

    for step_index, (start_yaw, end_yaw) in enumerate(zip(yaw_sequence[:-1], yaw_sequence[1:])):
        delta = wrap_angle(end_yaw - start_yaw)
        if abs(delta) > MAX_YAW_STEP + ANGLE_TOLERANCE:
            return {
                "step_index": step_index,
                "start_label": yaw_labels[step_index],
                "end_label": yaw_labels[step_index + 1],
                "start_yaw": start_yaw,
                "end_yaw": end_yaw,
                "delta": delta,
            }

    return None


def closed_loop_yaw_violation_message(violation):
    if violation["step_index"] == 0:
        return "The first waypoint yaw differs from the root yaw by more than {:.0f} degrees.".format(
            MAX_YAW_STEP_DEGREES
        )
    if violation["end_label"] == "ROOT":
        return "The final waypoint yaw differs from the root yaw by more than {:.0f} degrees on the return-to-ROOT leg.".format(
            MAX_YAW_STEP_DEGREES
        )
    return "Adjacent waypoint yaws differ by more than {:.0f} degrees between {} and {}.".format(
        MAX_YAW_STEP_DEGREES,
        violation["start_label"],
        violation["end_label"],
    )


def rounded_value(value):
    rounded = round(float(value), ROUND_DIGITS)
    if abs(rounded) < 0.5 * 10 ** (-ROUND_DIGITS):
        return 0.0
    return rounded


def format_float(value):
    text = f"{rounded_value(value):.{ROUND_DIGITS}f}"
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    if "." not in text:
        text += ".0"
    if text == "-0.0":
        return "0.0"
    return text


def build_segment_directions(ordered_positions):
    knot_positions = [ROOT_POSITION]
    knot_positions.extend(ordered_positions)
    knot_positions.append(ROOT_POSITION)

    segment_directions = []
    for start_position, end_position in zip(knot_positions[:-1], knot_positions[1:]):
        direction = unit_vector(subtract(end_position, start_position))
        if direction is None:
            return None
        segment_directions.append(direction)
    return segment_directions


def build_candidate_waypoints(ordered_positions, pitches):
    segment_directions = build_segment_directions(ordered_positions)
    if segment_directions is None:
        return None

    if dot(ROOT_FORWARD_AXIS, segment_directions[0]) <= DOT_EPSILON:
        return None
    if dot(ROOT_FORWARD_AXIS, segment_directions[-1]) <= DOT_EPSILON:
        return None

    waypoints = []
    waypoint_yaws = []

    for waypoint_index, position in enumerate(ordered_positions):
        incoming_direction = segment_directions[waypoint_index]
        outgoing_direction = segment_directions[waypoint_index + 1]
        yaw_x = incoming_direction[0] + outgoing_direction[0]
        yaw_y = incoming_direction[1] + outgoing_direction[1]
        if yaw_x * yaw_x + yaw_y * yaw_y <= SEGMENT_EPSILON:
            return None

        pitch = pitches[waypoint_index]
        yaw = math.atan2(yaw_y, yaw_x)
        forward_axis = forward_axis_from_pitch_yaw(pitch, yaw)

        if dot(forward_axis, incoming_direction) <= DOT_EPSILON:
            return None
        if dot(forward_axis, outgoing_direction) <= DOT_EPSILON:
            return None

        waypoints.append(
            [
                position[0],
                position[1],
                position[2],
                ROLL,
                pitch,
                yaw,
            ]
        )
        waypoint_yaws.append(yaw)

    if find_closed_loop_yaw_step_violation(waypoint_yaws) is not None:
        return None

    return [[rounded_value(value) for value in waypoint] for waypoint in waypoints]


def validate_generated_case(waypoints, circle_center, radius):
    if len(waypoints) != WAYPOINT_COUNT:
        raise ValueError(f"Expected {WAYPOINT_COUNT} waypoints, got {len(waypoints)}.")

    z_value = waypoints[0][2]
    ordered_positions = []
    waypoint_angles = []
    waypoint_yaws = []

    root_xy_distance = xy_distance(ROOT_POSITION, circle_center)
    if not math.isclose(root_xy_distance, radius, rel_tol=0.0, abs_tol=CIRCLE_TOLERANCE):
        raise ValueError("ROOT_POSITION is not on the waypoint circle in the XY plane.")

    for waypoint_index, waypoint in enumerate(waypoints):
        if len(waypoint) != 6:
            raise ValueError(f"Waypoint {waypoint_index} does not have 6 values.")

        x_value, y_value, z_current, roll, pitch, yaw = waypoint
        if not math.isclose(roll, ROLL, rel_tol=0.0, abs_tol=ANGLE_TOLERANCE):
            raise ValueError(f"Waypoint {waypoint_index} roll is not {ROLL}.")
        if not (PITCH_RANGE[0] - ANGLE_TOLERANCE <= pitch <= PITCH_RANGE[1] + ANGLE_TOLERANCE):
            raise ValueError(f"Waypoint {waypoint_index} pitch {pitch} is outside {PITCH_RANGE}.")
        if not math.isclose(z_current, z_value, rel_tol=0.0, abs_tol=CIRCLE_TOLERANCE):
            raise ValueError("Waypoints do not share the same z coordinate.")
        if y_value <= ROOT_POSITION[1] + HALF_SPACE_EPSILON:
            raise ValueError(f"Waypoint {waypoint_index} is not in the y > ROOT_POSITION[1] half-space.")
        if xy_distance((x_value, y_value), ROOT_POSITION) <= ROOT_XY_EXCLUSION_RADIUS + CIRCLE_TOLERANCE:
            raise ValueError(
                f"Waypoint {waypoint_index} falls within the XY exclusion radius of {ROOT_XY_EXCLUSION_RADIUS} m."
            )

        distance_to_center = xy_distance((x_value, y_value), circle_center)
        if not math.isclose(distance_to_center, radius, rel_tol=0.0, abs_tol=CIRCLE_TOLERANCE):
            raise ValueError(f"Waypoint {waypoint_index} is not on the requested circle.")

        current_angle = point_angle_on_circle((x_value, y_value), circle_center)
        for other_index, other_angle in enumerate(waypoint_angles):
            if angular_distance(current_angle, other_angle) <= MIN_WAYPOINT_ANGULAR_SEPARATION + ANGLE_TOLERANCE:
                raise ValueError(
                    "Waypoint {} and waypoint {} violate the minimum angular separation of {:.1f} degrees.".format(
                        other_index,
                        waypoint_index,
                        math.degrees(MIN_WAYPOINT_ANGULAR_SEPARATION),
                    )
                )

        ordered_positions.append((x_value, y_value, z_current))
        waypoint_angles.append(current_angle)
        waypoint_yaws.append(yaw)

    yaw_violation = find_closed_loop_yaw_step_violation(waypoint_yaws)
    if yaw_violation is not None:
        raise ValueError(closed_loop_yaw_violation_message(yaw_violation))

    segment_directions = build_segment_directions(ordered_positions)
    if segment_directions is None:
        raise ValueError("Generated waypoints contain a zero-length segment.")

    if dot(ROOT_FORWARD_AXIS, segment_directions[0]) <= DOT_EPSILON:
        raise ValueError("Root forward axis is incompatible with the first segment.")
    if dot(ROOT_FORWARD_AXIS, segment_directions[-1]) <= DOT_EPSILON:
        raise ValueError("Root forward axis is incompatible with the last segment.")

    for waypoint_index, waypoint in enumerate(waypoints):
        pitch = waypoint[4]
        yaw = waypoint[5]
        forward_axis = forward_axis_from_pitch_yaw(pitch, yaw)
        if dot(forward_axis, segment_directions[waypoint_index]) <= DOT_EPSILON:
            raise ValueError(f"Waypoint {waypoint_index} faces away from its incoming segment.")
        if dot(forward_axis, segment_directions[waypoint_index + 1]) <= DOT_EPSILON:
            raise ValueError(f"Waypoint {waypoint_index} faces away from its outgoing segment.")


def sample_waypoint_positions(rng, circle_center, radius, center_z):
    positions = []
    sampled_angles = []

    for _attempt in range(MAX_POSITION_SAMPLING_ATTEMPTS):
        angle = rng.uniform(0.0, 2.0 * math.pi)
        x_value = circle_center[0] + radius * math.cos(angle)
        y_value = circle_center[1] + radius * math.sin(angle)
        if y_value <= ROOT_POSITION[1] + HALF_SPACE_EPSILON:
            continue
        if xy_distance((x_value, y_value), ROOT_POSITION) <= ROOT_XY_EXCLUSION_RADIUS + CIRCLE_TOLERANCE:
            continue
        if any(
            angular_distance(angle, existing_angle) <= MIN_WAYPOINT_ANGULAR_SEPARATION + ANGLE_TOLERANCE
            for existing_angle in sampled_angles
        ):
            continue

        candidate = (x_value, y_value, center_z)
        if any(
            math.dist(candidate, existing_position) <= CIRCLE_TOLERANCE
            for existing_position in positions
        ):
            continue

        positions.append(candidate)
        sampled_angles.append(angle)
        if len(positions) == WAYPOINT_COUNT:
            return positions

    return None


def sample_circle_points(rng):
    diameter = rng.uniform(*DIAMETER_RANGE)
    radius = 0.5 * diameter
    center_angle = rng.uniform(*CENTER_ANGLE_RANGE)
    center_x = ROOT_POSITION[0] + radius * math.cos(center_angle)
    center_y = ROOT_POSITION[1] + radius * math.sin(center_angle)
    center_z = rng.uniform(*CIRCLE_Z_RANGE)
    positions = sample_waypoint_positions(rng, (center_x, center_y), radius, center_z)
    if positions is None:
        return None
    return (center_x, center_y, center_z), radius, positions


def generate_waypoint_case(rng):
    for _attempt in range(MAX_ATTEMPTS_PER_CASE):
        sampled_circle = sample_circle_points(rng)
        if sampled_circle is None:
            continue

        circle_center, radius, positions = sampled_circle
        pitches = [rng.uniform(*PITCH_RANGE) for _ in range(WAYPOINT_COUNT)]

        for ordered_positions in itertools.permutations(positions):
            waypoints = build_candidate_waypoints(ordered_positions, pitches)
            if waypoints is None:
                continue

            validate_generated_case(waypoints, circle_center, radius)
            return waypoints

    raise RuntimeError(
        f"Failed to generate a feasible {WAYPOINT_COUNT}-waypoint case within {MAX_ATTEMPTS_PER_CASE} attempts."
    )


def output_directory():
    return MONO_PLANNER_DIR / OUTPUT_DIR


def clean_output_directory(path):
    removed_count = 0
    for file_path in path.glob(f"{FILE_PREFIX}*.yaml"):
        file_path.unlink()
        removed_count += 1
    return removed_count


def write_case_file(file_path, waypoints):
    lines = [f"frame_id: {FRAME_ID}", "waypoints:"]
    for waypoint in waypoints:
        waypoint_text = ", ".join(format_float(value) for value in waypoint)
        lines.append(f"  - [{waypoint_text}]")
    file_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    rng = random.Random(RANDOM_SEED)
    target_directory = output_directory()
    target_directory.mkdir(parents=True, exist_ok=True)
    removed_count = clean_output_directory(target_directory)

    filename_width = max(4, len(str(NUM_CASES)))
    for case_index in range(1, NUM_CASES + 1):
        waypoints = generate_waypoint_case(rng)
        file_name = f"{FILE_PREFIX}{case_index:0{filename_width}d}.yaml"
        write_case_file(target_directory / file_name, waypoints)

    print(
        f"Generated {NUM_CASES} waypoint testcases in {target_directory} "
        f"(removed {removed_count} existing {FILE_PREFIX}*.yaml files first)."
    )


if __name__ == "__main__":
    main()
