#!/usr/bin/env python3

"""Analyze CopilotPlanner planning-time logs recorded in rosbag /rosout."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence

try:
    import rosbag
except ImportError as exc:
    raise SystemExit(
        "Failed to import rosbag. Source the ROS environment before running this script."
    ) from exc


ROSOUT_TOPIC = "rosout"
PLANNING_TIME_PATTERN = re.compile(
    r"\[CopilotPlanner\]\s+Joint cmd planning time:\s+([0-9]+(?:\.[0-9]+)?)\s+ms"
)


@dataclass(frozen=True)
class PlanningTimeStats:
    name: str
    sample_count: int
    min_ms: Optional[float]
    max_ms: Optional[float]
    mean_ms: Optional[float]


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    package_dir = script_dir.parent

    parser = argparse.ArgumentParser(
        description=(
            "Compute min/max/mean CopilotPlanner joint-command planning time "
            "from rosbag /rosout logs."
        )
    )
    parser.add_argument(
        "--bag-dir",
        type=Path,
        default=package_dir / "data" / "rosbag",
        help="Directory containing .bag files to analyze.",
    )
    return parser.parse_args(argv)


def iter_bag_files(bag_dir: Path) -> Iterable[Path]:
    if not bag_dir.exists():
        raise FileNotFoundError(f"bag directory does not exist: {bag_dir}")
    if not bag_dir.is_dir():
        raise NotADirectoryError(f"bag path is not a directory: {bag_dir}")
    return sorted(path for path in bag_dir.iterdir() if path.suffix == ".bag")


def read_planning_times_ms(bag_path: Path) -> List[float]:
    values = []
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, _ in bag.read_messages(topics=[ROSOUT_TOPIC]):
            match = PLANNING_TIME_PATTERN.search(msg.msg)
            if match:
                values.append(float(match.group(1)))
    return values


def compute_stats(name: str, values: Sequence[float]) -> PlanningTimeStats:
    if not values:
        return PlanningTimeStats(
            name=name,
            sample_count=0,
            min_ms=None,
            max_ms=None,
            mean_ms=None,
        )

    return PlanningTimeStats(
        name=name,
        sample_count=len(values),
        min_ms=min(values),
        max_ms=max(values),
        mean_ms=sum(values) / len(values),
    )


def format_ms(value: Optional[float]) -> str:
    if value is None:
        return "N/A"
    return f"{value:.3f}"


def format_row(stats: PlanningTimeStats, name_width: int) -> str:
    return (
        f"{stats.name:<{name_width}}  "
        f"{stats.sample_count:>7d}  "
        f"{format_ms(stats.min_ms):>8}  "
        f"{format_ms(stats.max_ms):>8}  "
        f"{format_ms(stats.mean_ms):>8}"
    )


def print_stats_table(stats_rows: Sequence[PlanningTimeStats]) -> None:
    name_width = max(len("bag"), *(len(stats.name) for stats in stats_rows))
    header = (
        f"{'bag':<{name_width}}  "
        f"{'samples':>7}  "
        f"{'min_ms':>8}  "
        f"{'max_ms':>8}  "
        f"{'mean_ms':>8}"
    )
    print(header)
    print("-" * len(header))
    for stats in stats_rows:
        print(format_row(stats, name_width))


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    bag_files = list(iter_bag_files(args.bag_dir))
    if not bag_files:
        print(f"No .bag files found in {args.bag_dir}", file=sys.stderr)
        return 1

    stats_rows = []
    combined_values = []
    for bag_path in bag_files:
        values = read_planning_times_ms(bag_path)
        stats_rows.append(compute_stats(bag_path.name, values))
        combined_values.extend(values)

    stats_rows.append(compute_stats("combined logged samples", combined_values))
    print_stats_table(stats_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
