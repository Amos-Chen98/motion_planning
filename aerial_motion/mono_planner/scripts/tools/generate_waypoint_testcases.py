#!/usr/bin/env python3

import itertools
import math
import random
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Tuple

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
MONO_PLANNER_DIR = SCRIPT_DIR.parent.parent
CONFIG_FILE = MONO_PLANNER_DIR / "config/waypoint_testcases/geo_only_sim.yaml"


@dataclass(frozen=True)
class GeneratorConfig:
    num_cases: int
    random_seed: int
    frame_id: str
    output_dir: Path
    file_prefix: str
    root_position: Tuple[float, float, float]
    root_forward_axis: Tuple[float, float, float]
    roll: float
    pitch_range: Tuple[float, float]
    x_range: Tuple[float, float]
    y_range: Tuple[float, float]
    z_range: Tuple[float, float]
    max_attempts_per_case: int
    waypoint_count: int
    round_digits: int
    segment_epsilon: float
    dot_epsilon: float
    angle_tolerance: float


def load_config(config_path):
    config_path = Path(config_path)
    try:
        with config_path.open(encoding="utf-8") as handle:
            values = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"Cannot load generator config '{config_path}': {exc}") from exc

    def invalid(field, message):
        raise ValueError(f"Generator config '{config_path}', field '{field}': {message}")

    if not isinstance(values, dict):
        invalid("<root>", "expected a YAML mapping.")

    expected_fields = {field.name for field in fields(GeneratorConfig)}
    for field in sorted(expected_fields):
        if field not in values:
            invalid(field, "required field is missing.")
    for field in values:
        if field not in expected_fields:
            invalid(field, "unknown field.")

    for field in ("num_cases", "random_seed", "max_attempts_per_case", "waypoint_count", "round_digits"):
        if type(values[field]) is not int:
            invalid(field, "expected an integer.")
        if field != "random_seed":
            minimum = 0 if field == "round_digits" else 1
            if values[field] < minimum:
                invalid(field, f"must be at least {minimum}.")

    for field in ("frame_id", "output_dir", "file_prefix"):
        if not isinstance(values[field], str) or not values[field].strip():
            invalid(field, "expected a non-empty string.")
    if any(character in values["file_prefix"] for character in "/\\*?[]"):
        invalid("file_prefix", "must be a filename prefix without path separators or glob patterns.")

    def finite_number(value, field):
        if type(value) not in (int, float):
            invalid(field, "expected a finite number.")
        try:
            number = float(value)
        except OverflowError:
            invalid(field, "expected a finite number.")
        if not math.isfinite(number):
            invalid(field, "expected a finite number.")
        return number

    for field in ("roll", "segment_epsilon", "dot_epsilon", "angle_tolerance"):
        values[field] = finite_number(values[field], field)
        if field != "roll" and values[field] <= 0.0:
            invalid(field, "must be greater than zero.")

    for field in ("root_position", "root_forward_axis", "pitch_range", "x_range", "y_range", "z_range"):
        size = 3 if field in ("root_position", "root_forward_axis") else 2
        if not isinstance(values[field], list) or len(values[field]) != size:
            invalid(field, f"expected a list of {size} numbers.")
        values[field] = tuple(
            finite_number(value, f"{field}[{index}]") for index, value in enumerate(values[field])
        )
        if size == 2 and values[field][0] > values[field][1]:
            invalid(field, "lower bound must not exceed upper bound.")
    if all(component == 0.0 for component in values["root_forward_axis"]):
        invalid("root_forward_axis", "must be a nonzero vector.")

    values["output_dir"] = Path(values["output_dir"])
    return GeneratorConfig(**values)


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def dot(left, right):
    return sum(left_value * right_value for left_value, right_value in zip(left, right))


def subtract(left, right):
    return tuple(left_value - right_value for left_value, right_value in zip(left, right))


def norm(vector):
    return math.sqrt(dot(vector, vector))


def unit_vector(vector, config):
    vector_norm = norm(vector)
    if vector_norm <= config.segment_epsilon:
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


def rounded_value(value, config):
    rounded = round(float(value), config.round_digits)
    if abs(rounded) < 0.5 * 10 ** (-config.round_digits):
        return 0.0
    return rounded


def format_float(value, config):
    text = f"{rounded_value(value, config):.{config.round_digits}f}"
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    if "." not in text:
        text += ".0"
    if text == "-0.0":
        return "0.0"
    return text


def build_segment_directions(ordered_positions, config):
    knot_positions = [config.root_position]
    knot_positions.extend(ordered_positions)
    knot_positions.append(config.root_position)

    segment_directions = []
    for start_position, end_position in zip(knot_positions[:-1], knot_positions[1:]):
        direction = unit_vector(subtract(end_position, start_position), config)
        if direction is None:
            return None
        segment_directions.append(direction)
    return segment_directions


def validate_segment_directions(segment_directions, config):
    if segment_directions is None:
        return "Generated waypoints contain a zero-length segment."
    if dot(config.root_forward_axis, segment_directions[0]) <= config.dot_epsilon:
        return "Root forward axis is incompatible with the first segment."
    if dot(config.root_forward_axis, segment_directions[-1]) <= config.dot_epsilon:
        return "Root forward axis is incompatible with the last segment."
    return None


def yaw_from_segment_bisector(incoming_direction, outgoing_direction, config):
    yaw_x = incoming_direction[0] + outgoing_direction[0]
    yaw_y = incoming_direction[1] + outgoing_direction[1]
    if yaw_x * yaw_x + yaw_y * yaw_y <= config.segment_epsilon:
        return None
    return math.atan2(yaw_y, yaw_x)


def validate_waypoint_forward_axis(forward_axis, incoming_direction, outgoing_direction, waypoint_index, config):
    if dot(forward_axis, incoming_direction) <= config.dot_epsilon:
        return f"Waypoint {waypoint_index} faces away from its incoming segment."
    if dot(forward_axis, outgoing_direction) <= config.dot_epsilon:
        return f"Waypoint {waypoint_index} faces away from its outgoing segment."
    return None


def build_nonholonomic_waypoints(ordered_positions, pitches, config):
    if len(ordered_positions) != len(pitches):
        raise ValueError("ordered_positions and pitches must have the same length.")

    segment_directions = build_segment_directions(ordered_positions, config)
    violation = validate_segment_directions(segment_directions, config)
    if violation is not None:
        return None, violation

    waypoints = []
    for waypoint_index, (position, pitch) in enumerate(zip(ordered_positions, pitches)):
        incoming_direction = segment_directions[waypoint_index]
        outgoing_direction = segment_directions[waypoint_index + 1]
        yaw = yaw_from_segment_bisector(incoming_direction, outgoing_direction, config)
        if yaw is None:
            return None, f"Waypoint {waypoint_index} does not have a well-defined horizontal segment bisector."

        forward_axis = forward_axis_from_pitch_yaw(pitch, yaw)
        violation = validate_waypoint_forward_axis(
            forward_axis,
            incoming_direction,
            outgoing_direction,
            waypoint_index,
            config,
        )
        if violation is not None:
            return None, violation

        waypoints.append([position[0], position[1], position[2], config.roll, pitch, yaw])

    return waypoints, None


def build_candidate_waypoints(ordered_positions, pitches, config):
    waypoints, _violation = build_nonholonomic_waypoints(ordered_positions, pitches, config)
    if waypoints is None:
        return None

    return [[rounded_value(value, config) for value in waypoint] for waypoint in waypoints]


def validate_generated_case(waypoints, config):
    if len(waypoints) != config.waypoint_count:
        raise ValueError(f"Expected {config.waypoint_count} waypoints, got {len(waypoints)}.")

    ordered_positions = []
    pitches = []

    for waypoint_index, waypoint in enumerate(waypoints):
        if len(waypoint) != 6:
            raise ValueError(f"Waypoint {waypoint_index} does not have 6 values.")

        x_value, y_value, z_value, roll, pitch = waypoint[:5]
        if not math.isclose(roll, config.roll, rel_tol=0.0, abs_tol=config.angle_tolerance):
            raise ValueError(f"Waypoint {waypoint_index} roll is not {config.roll}.")
        if not (config.pitch_range[0] - config.angle_tolerance <= pitch <= config.pitch_range[1] + config.angle_tolerance):
            raise ValueError(f"Waypoint {waypoint_index} pitch {pitch} is outside {config.pitch_range}.")

        ordered_positions.append((x_value, y_value, z_value))
        pitches.append(pitch)

    expected_waypoints, violation = build_nonholonomic_waypoints(ordered_positions, pitches, config)
    if violation is not None:
        raise ValueError(violation)

    for waypoint_index, (waypoint, expected_waypoint) in enumerate(zip(waypoints, expected_waypoints)):
        if angular_distance(waypoint[5], expected_waypoint[5]) > config.angle_tolerance:
            raise ValueError(
                f"Waypoint {waypoint_index} yaw {waypoint[5]} does not match the segment-bisector construction."
            )


def sample_waypoint_positions(rng, config):
    return [
        (
            rng.uniform(*config.x_range),
            rng.uniform(*config.y_range),
            rng.uniform(*config.z_range),
        )
        for _ in range(config.waypoint_count)
    ]


def generate_waypoint_case(rng, config):
    for _attempt in range(config.max_attempts_per_case):
        positions = sample_waypoint_positions(rng, config)
        pitches = [rng.uniform(*config.pitch_range) for _ in range(config.waypoint_count)]

        for ordered_positions in itertools.permutations(positions):
            waypoints = build_candidate_waypoints(ordered_positions, pitches, config)
            if waypoints is None:
                continue

            try:
                validate_generated_case(waypoints, config)
            except ValueError:
                continue
            return waypoints

    raise RuntimeError(
        f"Failed to generate a feasible {config.waypoint_count}-waypoint case "
        f"within {config.max_attempts_per_case} attempts."
    )


def output_directory(config):
    return MONO_PLANNER_DIR / config.output_dir


def clean_output_directory(path, config):
    removed_count = 0
    for file_path in path.glob(f"{config.file_prefix}*.yaml"):
        file_path.unlink()
        removed_count += 1
    return removed_count


def write_case_file(file_path, waypoints, config):
    lines = [f"frame_id: {config.frame_id}", "waypoints:"]
    for waypoint in waypoints:
        waypoint_text = ", ".join(format_float(value, config) for value in waypoint)
        lines.append(f"  - [{waypoint_text}]")
    file_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    config = load_config(CONFIG_FILE)
    rng = random.Random(config.random_seed)
    target_directory = output_directory(config)
    target_directory.mkdir(parents=True, exist_ok=True)
    removed_count = clean_output_directory(target_directory, config)

    filename_width = max(4, len(str(config.num_cases)))
    for case_index in range(1, config.num_cases + 1):
        waypoints = generate_waypoint_case(rng, config)
        file_name = f"{config.file_prefix}{case_index:0{filename_width}d}.yaml"
        write_case_file(target_directory / file_name, waypoints, config)

    print(
        f"Generated {config.num_cases} waypoint testcases in {target_directory} "
        f"(removed {removed_count} existing {config.file_prefix}*.yaml files first)."
    )


if __name__ == "__main__":
    main()
