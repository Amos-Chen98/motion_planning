#!/usr/bin/env python3

"""Plot six-dimensional joints_ctrl commands from one rosbag."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import rosbag


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent

# Manually edit these parameters before running the script.
BAG_PATH = PACKAGE_DIR / "data" / "rosbag" / "2026-05-06-18-14-00_four_ring_success.bag"
TOPIC = "/dragon/joints_ctrl"
DURATION_SECONDS = 50.0
OUTPUT_DIR = PACKAGE_DIR / "data" / "figures" / "joint_cmd"
OUTPUT_SUFFIX = "joint_cmd"
OUTPUT_LEGEND_SUFFIX = "joint_cmd_legend"
SAVE_FIGURE = True
SHOW_PLOT = True
SAVE_DPI = 200
IEEE_DOUBLE_COLUMN_TEXT_WIDTH_INCH = 43.0 * 12.0 / 72.27
FIGURE_WIDTH_INCH = IEEE_DOUBLE_COLUMN_TEXT_WIDTH_INCH / 3.0
FIGURE_HEIGHT_INCH = FIGURE_WIDTH_INCH * 2.0 / 4.0
LEGEND_FIGURE_WIDTH_INCH = IEEE_DOUBLE_COLUMN_TEXT_WIDTH_INCH
LEGEND_FIGURE_HEIGHT_INCH = 0.34
FONT_SIZE_PT = 9
AXES_LEFT = 0.18
AXES_RIGHT = 0.965
AXES_BOTTOM = 0.295
AXES_TOP = 0.985
X_LABEL_PAD = 0.5
Y_LABEL_PAD = 0.2
TICK_PAD = 1.2
TICK_LENGTH = 2.4
TICK_WIDTH = 0.6
LEGEND_ANCHOR = (0.015, 0.02, 0.97, 0.96)
JOINT_COLORS = (
    "#4477AA",
    "#EE6677",
    "#228833",
    "#CCBB44",
    "#66CCEE",
    "#AA3377",
)

JOINT_LABELS = (
    "joint1 pitch",
    "joint1 yaw",
    "joint2 pitch",
    "joint2 yaw",
    "joint3 pitch",
    "joint3 yaw",
)


@dataclass(frozen=True)
class JointControlSeries:
    topic_names: tuple[str, ...]
    start_time: float
    requested_end_time: float
    actual_end_time: float
    times: list[float]
    positions: list[list[float]]
    exhausted_before_requested_end: bool


def normalized_topic_candidates(topic: str) -> tuple[str, ...]:
    stripped = topic.strip()
    if not stripped:
        raise RuntimeError("topic must not be empty")

    without_slash = stripped.lstrip("/")
    candidates = [stripped]
    if stripped.startswith("/"):
        candidates.append(without_slash)
    else:
        candidates.append("/" + stripped)

    unique_candidates = []
    for candidate in candidates:
        if candidate and candidate not in unique_candidates:
            unique_candidates.append(candidate)
    return tuple(unique_candidates)


def resolve_available_topics(bag: rosbag.Bag, requested_topic: str) -> tuple[str, ...]:
    available_topic_names = bag.get_type_and_topic_info().topics.keys()
    candidates = normalized_topic_candidates(requested_topic)
    resolved = tuple(candidate for candidate in candidates if candidate in available_topic_names)
    if resolved:
        return resolved

    raise RuntimeError(
        "topic not found in bag: {} (checked: {})".format(
            requested_topic,
            ", ".join(candidates),
        )
    )


def load_joint_control_series(
    bag_path: Path,
    requested_topic: str,
    duration: float,
) -> JointControlSeries:
    bag_path = Path(bag_path).expanduser().resolve()
    if not bag_path.is_file():
        raise RuntimeError("bag file does not exist: {}".format(bag_path))
    if duration <= 0.0:
        raise RuntimeError("DURATION_SECONDS must be greater than 0")

    times: list[float] = []
    positions: list[list[float]] = []
    start_time = None
    requested_end_time = None
    topic_names: tuple[str, ...]
    exhausted_before_requested_end = True

    with rosbag.Bag(str(bag_path), "r") as bag:
        topic_names = resolve_available_topics(bag, requested_topic)
        for topic_name, msg, bag_time in bag.read_messages(topics=topic_names):
            absolute_time = bag_time.to_sec()
            if start_time is None:
                start_time = absolute_time
                requested_end_time = start_time + duration

            relative_time = absolute_time - start_time
            if relative_time > duration:
                exhausted_before_requested_end = False
                break

            position = list(msg.position)
            if len(position) != len(JOINT_LABELS):
                raise RuntimeError(
                    "{} at bag time {:.6f} has {} position values, expected {}".format(
                        topic_name,
                        absolute_time,
                        len(position),
                        len(JOINT_LABELS),
                    )
                )

            times.append(relative_time)
            positions.append(position)

    if start_time is None or requested_end_time is None:
        raise RuntimeError("no messages found for topic: {}".format(requested_topic))

    return JointControlSeries(
        topic_names=topic_names,
        start_time=start_time,
        requested_end_time=requested_end_time,
        actual_end_time=start_time + times[-1],
        times=times,
        positions=positions,
        exhausted_before_requested_end=exhausted_before_requested_end,
    )


def import_pyplot(use_noninteractive_backend: bool):
    if use_noninteractive_backend:
        import matplotlib

        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    return plt


def configure_plot_style(plt) -> None:
    plt.rcParams.update(
        {
            "font.family": "Times New Roman",
            "font.serif": ["Times New Roman"],
            "font.size": FONT_SIZE_PT,
            "axes.labelsize": FONT_SIZE_PT,
            "axes.titlesize": FONT_SIZE_PT,
            "axes.facecolor": "white",
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
            "xtick.labelsize": FONT_SIZE_PT,
            "ytick.labelsize": FONT_SIZE_PT,
            "legend.fontsize": FONT_SIZE_PT,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "path",
        }
    )


def plot_joint_control_series(
    series: JointControlSeries,
    duration: float,
    plt,
):
    configure_plot_style(plt)

    fig, ax = plt.subplots(figsize=(FIGURE_WIDTH_INCH, FIGURE_HEIGHT_INCH))
    fig.subplots_adjust(
        left=AXES_LEFT,
        right=AXES_RIGHT,
        bottom=AXES_BOTTOM,
        top=AXES_TOP,
    )

    linestyles = ["-", "-", "-", "-", "-", "-"]
    series_by_joint = list(zip(*series.positions))
    for values, label, color, linestyle in zip(
        series_by_joint,
        JOINT_LABELS,
        JOINT_COLORS,
        linestyles,
    ):
        ax.plot(
            series.times,
            values,
            label=label,
            color=color,
            linestyle=linestyle,
            linewidth=1.0,
        )

    ax.axhline(0.0, color="0.35", linewidth=0.55, alpha=0.55)
    ax.grid(True, which="major", color="0.82", linewidth=0.4, alpha=0.7)
    ax.set_axisbelow(True)

    x_max = series.times[-1] if series.exhausted_before_requested_end else duration
    if x_max <= 0.0:
        x_max = 1.0

    ax.set_xlim(0.0, x_max)
    ax.set_xlabel("Time [s]", labelpad=X_LABEL_PAD)
    ax.set_ylabel("Joint command\n[rad]", labelpad=Y_LABEL_PAD)
    ax.tick_params(
        axis="both",
        which="major",
        pad=TICK_PAD,
        length=TICK_LENGTH,
        width=TICK_WIDTH,
    )
    return fig


def plot_joint_control_legend(plt):
    configure_plot_style(plt)

    from matplotlib.lines import Line2D

    fig = plt.figure(figsize=(LEGEND_FIGURE_WIDTH_INCH, LEGEND_FIGURE_HEIGHT_INCH))
    linestyles = ["-", "-", "-", "-", "-", "-"]
    handles = [
        Line2D([0], [0], color=color, linestyle=linestyle, linewidth=1.0)
        for color, linestyle in zip(JOINT_COLORS, linestyles)
    ]
    fig.legend(
        handles,
        JOINT_LABELS,
        loc="center",
        bbox_to_anchor=LEGEND_ANCHOR,
        mode="expand",
        ncol=len(JOINT_LABELS),
        frameon=True,
        framealpha=0.95,
        borderaxespad=0.0,
        borderpad=0.3,
        labelspacing=0.2,
        handlelength=1.2,
        handletextpad=0.8,
        columnspacing=1.0,
    )
    return fig


def resolve_output_svg_path(output_dir: Path, bag_path: Path, output_suffix: str) -> Path:
    directory = Path(output_dir).expanduser().resolve()
    bag_stem = Path(bag_path).expanduser().resolve().stem
    return directory / "{}_{}.svg".format(bag_stem, output_suffix)


def print_summary(series: JointControlSeries, duration: float) -> None:
    actual_duration = series.actual_end_time - series.start_time
    print(
        "Read {} messages from {}.".format(
            len(series.times),
            ", ".join(series.topic_names),
        )
    )
    print(
        "Plot window: bag time {:.6f} -> {:.6f}; relative {:.3f} s -> {:.3f} s.".format(
            series.start_time,
            series.actual_end_time,
            0.0,
            actual_duration,
        )
    )
    print(
        "Requested end: {:.3f} s after first message (bag time {:.6f}).".format(
            duration,
            series.requested_end_time,
        )
    )
    if series.exhausted_before_requested_end:
        print("Requested duration exceeds available topic data; plotted through the final message.")


def main() -> int:
    try:
        series = load_joint_control_series(BAG_PATH, TOPIC, DURATION_SECONDS)
        print("Bag: {}".format(Path(BAG_PATH).expanduser().resolve()))
        print_summary(series, DURATION_SECONDS)

        plt = import_pyplot(not SHOW_PLOT)
        fig = plot_joint_control_series(series, DURATION_SECONDS, plt)
        legend_fig = plot_joint_control_legend(plt)

        if SAVE_FIGURE:
            output_path = resolve_output_svg_path(OUTPUT_DIR, BAG_PATH, OUTPUT_SUFFIX)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            plt.rcParams["svg.fonttype"] = "path"
            fig.savefig(output_path, dpi=SAVE_DPI)
            print("Saved plot: {}".format(output_path))

            legend_output_path = resolve_output_svg_path(
                OUTPUT_DIR,
                BAG_PATH,
                OUTPUT_LEGEND_SUFFIX,
            )
            plt.rcParams["svg.fonttype"] = "path"
            legend_fig.savefig(legend_output_path, dpi=SAVE_DPI)
            print("Saved legend: {}".format(legend_output_path))

        if SHOW_PLOT:
            plt.show()

        plt.close(fig)
        plt.close(legend_fig)
        return 0
    except (RuntimeError, rosbag.ROSBagException) as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
