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
ROOT_POSITION = (0.5, 0.0, 1.0)
ROLL = 0.0
PITCH_RANGE = (-0.2, 0.2)
X_RANGE = (-2.0, 2.0)
Y_RANGE = (-2.0, 2.0)
Z_RANGE = (0.5, 4.0)
MAX_ATTEMPTS_PER_CASE = 1000

WAYPOINT_COUNT = 4
ROUND_DIGITS = 9
SEGMENT_EPSILON = 1e-9
DOT_EPSILON = 1e-9
ANGLE_TOLERANCE = 1e-9

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


def angular_distance(angle_a, angle_b):
    return abs(wrap_angle(angle_a - angle_b))


def forward_axis_from_pitch_yaw(pitch, yaw):
    cosine_pitch = math.cos(pitch)
    return (
        math.cos(yaw) * cosine_pitch,
        math.sin(yaw) * cosine_pitch,
        -math.sin(pitch),
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


def validate_segment_directions(segment_directions):
    if segment_directions is None:
        return "Generated waypoints contain a zero-length segment."
    if dot(ROOT_FORWARD_AXIS, segment_directions[0]) <= DOT_EPSILON:
        return "Root forward axis is incompatible with the first segment."
    if dot(ROOT_FORWARD_AXIS, segment_directions[-1]) <= DOT_EPSILON:
        return "Root forward axis is incompatible with the last segment."
    return None


def yaw_from_segment_bisector(incoming_direction, outgoing_direction):
    yaw_x = incoming_direction[0] + outgoing_direction[0]
    yaw_y = incoming_direction[1] + outgoing_direction[1]
    if yaw_x * yaw_x + yaw_y * yaw_y <= SEGMENT_EPSILON:
        return None
    return math.atan2(yaw_y, yaw_x)


def validate_waypoint_forward_axis(forward_axis, incoming_direction, outgoing_direction, waypoint_index):
    if dot(forward_axis, incoming_direction) <= DOT_EPSILON:
        return f"Waypoint {waypoint_index} faces away from its incoming segment."
    if dot(forward_axis, outgoing_direction) <= DOT_EPSILON:
        return f"Waypoint {waypoint_index} faces away from its outgoing segment."
    return None


def build_nonholonomic_waypoints(ordered_positions, pitches):
    if len(ordered_positions) != len(pitches):
        raise ValueError("ordered_positions and pitches must have the same length.")

    segment_directions = build_segment_directions(ordered_positions)
    violation = validate_segment_directions(segment_directions)
    if violation is not None:
        return None, violation

    waypoints = []
    for waypoint_index, (position, pitch) in enumerate(zip(ordered_positions, pitches)):
        incoming_direction = segment_directions[waypoint_index]
        outgoing_direction = segment_directions[waypoint_index + 1]
        yaw = yaw_from_segment_bisector(incoming_direction, outgoing_direction)
        if yaw is None:
            return None, f"Waypoint {waypoint_index} does not have a well-defined horizontal segment bisector."

        forward_axis = forward_axis_from_pitch_yaw(pitch, yaw)
        violation = validate_waypoint_forward_axis(
            forward_axis,
            incoming_direction,
            outgoing_direction,
            waypoint_index,
        )
        if violation is not None:
            return None, violation

        waypoints.append([position[0], position[1], position[2], ROLL, pitch, yaw])

    return waypoints, None


def build_candidate_waypoints(ordered_positions, pitches):
    waypoints, _violation = build_nonholonomic_waypoints(ordered_positions, pitches)
    if waypoints is None:
        return None

    return [[rounded_value(value) for value in waypoint] for waypoint in waypoints]


def validate_generated_case(waypoints):
    if len(waypoints) != WAYPOINT_COUNT:
        raise ValueError(f"Expected {WAYPOINT_COUNT} waypoints, got {len(waypoints)}.")

    ordered_positions = []
    pitches = []

    for waypoint_index, waypoint in enumerate(waypoints):
        if len(waypoint) != 6:
            raise ValueError(f"Waypoint {waypoint_index} does not have 6 values.")

        x_value, y_value, z_value, roll, pitch = waypoint[:5]
        if not math.isclose(roll, ROLL, rel_tol=0.0, abs_tol=ANGLE_TOLERANCE):
            raise ValueError(f"Waypoint {waypoint_index} roll is not {ROLL}.")
        if not (PITCH_RANGE[0] - ANGLE_TOLERANCE <= pitch <= PITCH_RANGE[1] + ANGLE_TOLERANCE):
            raise ValueError(f"Waypoint {waypoint_index} pitch {pitch} is outside {PITCH_RANGE}.")

        ordered_positions.append((x_value, y_value, z_value))
        pitches.append(pitch)

    expected_waypoints, violation = build_nonholonomic_waypoints(ordered_positions, pitches)
    if violation is not None:
        raise ValueError(violation)

    for waypoint_index, (waypoint, expected_waypoint) in enumerate(zip(waypoints, expected_waypoints)):
        if angular_distance(waypoint[5], expected_waypoint[5]) > ANGLE_TOLERANCE:
            raise ValueError(
                f"Waypoint {waypoint_index} yaw {waypoint[5]} does not match the segment-bisector construction."
            )


def sample_waypoint_positions(rng):
    return [
        (
            rng.uniform(*X_RANGE),
            rng.uniform(*Y_RANGE),
            rng.uniform(*Z_RANGE),
        )
        for _ in range(WAYPOINT_COUNT)
    ]


def generate_waypoint_case(rng):
    for _attempt in range(MAX_ATTEMPTS_PER_CASE):
        positions = sample_waypoint_positions(rng)
        pitches = [rng.uniform(*PITCH_RANGE) for _ in range(WAYPOINT_COUNT)]

        for ordered_positions in itertools.permutations(positions):
            waypoints = build_candidate_waypoints(ordered_positions, pitches)
            if waypoints is None:
                continue

            try:
                validate_generated_case(waypoints)
            except ValueError:
                continue
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
