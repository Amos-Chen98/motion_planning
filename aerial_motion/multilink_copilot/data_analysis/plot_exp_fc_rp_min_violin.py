#!/usr/bin/env python3

"""Replay experiment rosbags, recompute fc_rp_min, and plot violin summaries.

The metric definition is intentionally not reimplemented here.  Each bag is
replayed through the current C++ copilot_planner with
publish_stability_metrics:=true, so the recorded samples are the values from
CopilotPlanner::evaluateStability() in copilot_stability.cpp.
"""

from __future__ import annotations

import csv
import math
import os
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rosbag
import rosgraph
import rosnode
import rospy
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent

# Manually edit these parameters before running the script.
BAG_SPECS = (
    (
        "Scene (a)",
        PACKAGE_DIR
        / "data"
        / "rosbag"
        / "2026-05-25-17-44-14_close_ring_success.bag",
    ),
    (
        "Scene (b)",
        PACKAGE_DIR
        / "data"
        / "rosbag"
        / "2026-05-26-15-37-10_large_pitch_success.bag",
    ),
    (
        "Scene (c)",
        PACKAGE_DIR
        / "data"
        / "rosbag"
        / "2026-05-06-18-14-00_four_ring_success.bag",
    ),
)

ROBOT_NS = "dragon"
ROBOT_MODEL_XACRO = "$(find dragon)/robots/quad/v1_5_202601.urdf.xacro"
TARGET_POSE_FRAME_TYPE = "FLU"
ROSBAG_PLAY_RATE = 1.0
REQUIRED_BAG_TOPICS = (
    "dragon/joint_states",
    "dragon/root/target_pose",
)
JOINT_STATES_TOPIC = "/dragon/joint_states"
TARGET_POSE_TOPIC = "/dragon/root/target_pose"
METRIC_TOPIC = "/dragon/stability/fc_rp_min"

OUTPUT_DIR = PACKAGE_DIR / "data" / "figures" / "exp_fc_rp"
OUTPUT_SVG_PATH = OUTPUT_DIR / "exp_fc_rp_min_violin.svg"
OUTPUT_CSV_PATH = OUTPUT_DIR / "exp_fc_rp_min_samples.csv"
SAVE_DPI = 200

# IEEE Transactions two-column text width; a half-page figure spans one column.
IEEE_PAGE_WIDTH_INCH = 7.16
FIGURE_WIDTH_INCH = IEEE_PAGE_WIDTH_INCH / 2.0
# The SVG is saved with bbox_inches="tight" to remove external whitespace.  This
# slightly taller canvas yields a tight-cropped output ratio matching
# plot_link_tracking_separation.py: (7.16 / 2) / 1.76.
FIGURE_HEIGHT_INCH = 1.8126
FONT_SIZE_PT = 9
AXES_LEFT = 0.175
AXES_RIGHT = 0.985
AXES_BOTTOM = 0.20
AXES_TOP = 0.965
Y_LABEL = r"$d_{\mathrm{rp}}$ [\si{\newton\meter}]"
FC_RP_MIN_THRESHOLD = 3.2
THRESHOLD_LABEL = (
    r"Lower bound $\underline{d}_{\mathrm{rp}}="
    rf"\SI{{{FC_RP_MIN_THRESHOLD}}}{{\newton\meter}}$"
)

SCENE_COLORS = ("#0072B2", "#D55E00", "#009E73")
THRESHOLD_COLOR = "#CC79A7"
VIOLIN_ALPHA = 0.72
VIOLIN_WIDTH = 0.72
QUARTILE_LINE_WIDTH = 1.2
MEDIAN_LINE_WIDTH = 1.5

MASTER_WAIT_TIMEOUT_S = 15.0
LAUNCH_WAIT_TIMEOUT_S = 45.0
PLAYBACK_TIMEOUT_MARGIN_S = 60.0
POST_PLAYBACK_DRAIN_S = 0.4


@dataclass(frozen=True)
class BagSpec:
    scene_label: str
    bag_path: Path


@dataclass(frozen=True)
class MetricSample:
    scene_label: str
    bag_stem: str
    sample_index: int
    sim_time: float
    fc_rp_min: float


@dataclass(frozen=True)
class SceneMetricSeries:
    spec: BagSpec
    samples: tuple[MetricSample, ...]

    @property
    def values(self) -> tuple[float, ...]:
        return tuple(sample.fc_rp_min for sample in self.samples)


class MetricCollector:
    """Collect planner-published metric samples after input topics are observed."""

    def __init__(self, scene_label: str, bag_stem: str):
        self._scene_label = scene_label
        self._bag_stem = bag_stem
        self._lock = threading.Lock()
        self._joint_state_seen = False
        self._target_pose_seen = False
        self._samples: list[MetricSample] = []
        self._subscribers = (
            rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self._joint_state_callback, queue_size=10),
            rospy.Subscriber(TARGET_POSE_TOPIC, PoseStamped, self._target_pose_callback, queue_size=10),
            rospy.Subscriber(METRIC_TOPIC, Float64, self._metric_callback, queue_size=1000),
        )

    def unregister(self) -> None:
        for subscriber in self._subscribers:
            subscriber.unregister()

    def samples(self) -> tuple[MetricSample, ...]:
        with self._lock:
            return tuple(self._samples)

    def _joint_state_callback(self, _msg: JointState) -> None:
        with self._lock:
            self._joint_state_seen = True

    def _target_pose_callback(self, _msg: PoseStamped) -> None:
        with self._lock:
            self._target_pose_seen = True

    def _metric_callback(self, msg: Float64) -> None:
        value = float(msg.data)
        if not math.isfinite(value):
            return

        sim_time = rospy.Time.now().to_sec()
        with self._lock:
            if not (self._joint_state_seen and self._target_pose_seen):
                return

            self._samples.append(
                MetricSample(
                    scene_label=self._scene_label,
                    bag_stem=self._bag_stem,
                    sample_index=len(self._samples),
                    sim_time=sim_time,
                    fc_rp_min=value,
                )
            )


class ClockWarmupPublisher:
    """Publish temporary /clock messages so sim-time nodes can initialize."""

    def __init__(self, rate_hz: float = 20.0):
        self._period_s = 1.0 / rate_hz
        self._stop_event = threading.Event()
        self._publisher = rospy.Publisher("/clock", Clock, queue_size=10)
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        self._publisher.unregister()

    def _run(self) -> None:
        start_time = time.monotonic()
        while not self._stop_event.is_set():
            elapsed = time.monotonic() - start_time
            self._publisher.publish(Clock(clock=rospy.Time.from_sec(elapsed)))
            time.sleep(self._period_s)


def normalize_topic_candidates(topic: str) -> tuple[str, ...]:
    stripped = topic.strip()
    if not stripped:
        raise RuntimeError("topic must not be empty")

    without_slash = stripped.lstrip("/")
    candidates = (without_slash, "/" + without_slash)
    unique_candidates: list[str] = []
    for candidate in candidates:
        if candidate and candidate not in unique_candidates:
            unique_candidates.append(candidate)
    return tuple(unique_candidates)


def resolve_required_bag_topics(bag_path: Path) -> tuple[str, ...]:
    with rosbag.Bag(str(bag_path), "r") as bag:
        available_topics = set(bag.get_type_and_topic_info().topics.keys())

    resolved_topics: list[str] = []
    for topic in REQUIRED_BAG_TOPICS:
        for candidate in normalize_topic_candidates(topic):
            if candidate in available_topics:
                resolved_topics.append(candidate)
                break
        else:
            raise RuntimeError(
                "{} does not contain required topic {} (checked: {})".format(
                    bag_path,
                    topic,
                    ", ".join(normalize_topic_candidates(topic)),
                )
            )

    return tuple(resolved_topics)


def bag_duration_seconds(bag_path: Path) -> float:
    with rosbag.Bag(str(bag_path), "r") as bag:
        return max(0.0, bag.get_end_time() - bag.get_start_time())


def master() -> rosgraph.Master:
    return rosgraph.Master("/plot_exp_fc_rp_min_violin")


def is_master_online() -> bool:
    try:
        return bool(master().is_online())
    except Exception:
        return False


def wait_for_master(timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if is_master_online():
            return
        time.sleep(0.2)
    raise RuntimeError("ROS master did not become available within {:.1f} s".format(timeout_s))


def start_roscore_if_needed() -> subprocess.Popen | None:
    if is_master_online():
        return None

    process = subprocess.Popen(
        ["roscore"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    try:
        wait_for_master(MASTER_WAIT_TIMEOUT_S)
    except Exception:
        stop_process_group(process)
        raise
    return process


def ensure_clean_ros_graph() -> None:
    try:
        node_names = rosnode.get_node_names()
    except Exception as exc:
        raise RuntimeError("failed to inspect ROS nodes: {}".format(exc)) from exc

    conflicts: list[str] = []
    if "/dragon/copilot_planner" in node_names:
        conflicts.append("/dragon/copilot_planner")

    for node_name in node_names:
        base_name = node_name.rsplit("/", 1)[-1]
        if base_name == "rosbag_play" or base_name.startswith("play_"):
            conflicts.append(node_name)

    if conflicts:
        raise RuntimeError(
            "refusing to run with existing planner/rosbag-play nodes: {}".format(
                ", ".join(sorted(conflicts))
            )
        )


def set_use_sim_time() -> None:
    master().setParam("/use_sim_time", True)


def initialize_rospy_node() -> None:
    if rospy.core.is_initialized():
        return
    rospy.init_node("plot_exp_fc_rp_min_violin", anonymous=True, disable_signals=True)


def write_temp_launch_file() -> tempfile.NamedTemporaryFile:
    launch_text = f"""<?xml version="1.0"?>
<launch>
    <param name="/use_sim_time" value="true" />

    <group ns="{ROBOT_NS}">
        <param name="robot_description" command="$(find xacro)/xacro '{ROBOT_MODEL_XACRO}' robot_name:={ROBOT_NS}" />

        <node pkg="multilink_copilot" type="copilot_planner" name="copilot_planner" output="screen">
            <rosparam command="load" file="$(find multilink_copilot)/config/copilot_planner.yaml" />
            <param name="target_pose_frame_type" value="{TARGET_POSE_FRAME_TYPE}" />
            <param name="publish_stability_metrics" value="true" />
            <param name="verbose" value="false" />
        </node>
    </group>
</launch>
"""
    launch_file = tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        prefix="exp_fc_rp_min_",
        suffix=".launch",
        delete=False,
    )
    with launch_file:
        launch_file.write(launch_text)
    return launch_file


def start_copilot_launch() -> tuple[subprocess.Popen, Path]:
    launch_file = write_temp_launch_file()
    launch_path = Path(launch_file.name)
    process = subprocess.Popen(
        ["roslaunch", str(launch_path)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return process, launch_path


def process_exited(process: subprocess.Popen) -> bool:
    return process.poll() is not None


def wait_for_node(node_name: str, process: subprocess.Popen, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process_exited(process):
            raise RuntimeError(
                "roslaunch exited before {} was available (return code {})".format(
                    node_name,
                    process.returncode,
                )
            )
        try:
            if node_name in rosnode.get_node_names():
                return
        except Exception:
            pass
        time.sleep(0.2)
    raise RuntimeError("{} did not become available within {:.1f} s".format(node_name, timeout_s))


def metric_publisher_count() -> int:
    try:
        publications, _subscriptions, _services = master().getSystemState()
    except Exception as exc:
        raise RuntimeError("failed to inspect ROS graph: {}".format(exc)) from exc

    for topic_name, publishers in publications:
        if topic_name == METRIC_TOPIC:
            return len(publishers)
    return 0


def wait_for_single_metric_publisher(process: subprocess.Popen, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process_exited(process):
            raise RuntimeError(
                "roslaunch exited before {} was advertised (return code {})".format(
                    METRIC_TOPIC,
                    process.returncode,
                )
            )

        count = metric_publisher_count()
        if count == 1:
            return
        if count > 1:
            raise RuntimeError(
                "{} has {} publishers; expected exactly one copilot_planner publisher".format(
                    METRIC_TOPIC,
                    count,
                )
            )
        time.sleep(0.2)

    raise RuntimeError("{} was not advertised within {:.1f} s".format(METRIC_TOPIC, timeout_s))


def stop_process_group(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return

    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return

    try:
        process.wait(timeout=10.0)
        return
    except subprocess.TimeoutExpired:
        pass

    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return

    try:
        process.wait(timeout=5.0)
        return
    except subprocess.TimeoutExpired:
        pass

    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    process.wait(timeout=5.0)


def remove_temp_launch_file(path: Path | None) -> None:
    if path is None:
        return
    try:
        path.unlink()
    except FileNotFoundError:
        pass


def play_bag_inputs(bag_path: Path, topics: tuple[str, ...]) -> None:
    timeout_s = bag_duration_seconds(bag_path) / ROSBAG_PLAY_RATE + PLAYBACK_TIMEOUT_MARGIN_S
    command = [
        "rosbag",
        "play",
        "--clock",
        "--quiet",
        "--wait-for-subscribers",
        "--rate",
        "{:g}".format(ROSBAG_PLAY_RATE),
        "--topics",
        *topics,
        "--bags",
        str(bag_path),
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )

    try:
        process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired as exc:
        stop_process_group(process)
        raise RuntimeError(
            "rosbag play timed out after {:.1f} s for {}".format(timeout_s, bag_path)
        ) from exc

    if process.returncode != 0:
        raise RuntimeError(
            "rosbag play failed for {} with return code {}".format(bag_path, process.returncode)
        )


def replay_and_collect(spec: BagSpec) -> SceneMetricSeries:
    bag_path = spec.bag_path.expanduser().resolve()
    if not bag_path.is_file():
        raise RuntimeError("bag file does not exist: {}".format(bag_path))

    play_topics = resolve_required_bag_topics(bag_path)
    launch_process: subprocess.Popen | None = None
    launch_path: Path | None = None
    clock_warmup: ClockWarmupPublisher | None = None
    collector: MetricCollector | None = None

    print("Recomputing {} from {}".format(spec.scene_label, bag_path), flush=True)
    print("  playing input topics only: {}".format(", ".join(play_topics)), flush=True)

    try:
        launch_process, launch_path = start_copilot_launch()
        clock_warmup = ClockWarmupPublisher()
        clock_warmup.start()
        wait_for_node("/dragon/copilot_planner", launch_process, LAUNCH_WAIT_TIMEOUT_S)
        wait_for_single_metric_publisher(launch_process, LAUNCH_WAIT_TIMEOUT_S)
        clock_warmup.stop()
        clock_warmup = None

        collector = MetricCollector(spec.scene_label, bag_path.stem)
        time.sleep(0.2)
        play_bag_inputs(bag_path, play_topics)
        time.sleep(POST_PLAYBACK_DRAIN_S)

        samples = collector.samples()
        if not samples:
            raise RuntimeError("no {} samples collected for {}".format(METRIC_TOPIC, bag_path))

        return SceneMetricSeries(spec=BagSpec(spec.scene_label, bag_path), samples=samples)
    finally:
        if collector is not None:
            collector.unregister()
        if clock_warmup is not None:
            clock_warmup.stop()
        stop_process_group(launch_process)
        remove_temp_launch_file(launch_path)


def write_samples_csv(series_list: tuple[SceneMetricSeries, ...], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.writer(output_file, lineterminator="\n")
        writer.writerow(("scene_label", "bag_stem", "sample_index", "sim_time", "fc_rp_min"))
        for series in series_list:
            for sample in series.samples:
                writer.writerow(
                    (
                        sample.scene_label,
                        sample.bag_stem,
                        sample.sample_index,
                        "{:.9f}".format(sample.sim_time),
                        "{:.12g}".format(sample.fc_rp_min),
                    )
                )


def import_pyplot():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def configure_plot_style(plt) -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": [
                "Nimbus Roman",
                "Nimbus Roman No9 L",
                "Times New Roman",
                "Times",
            ],
            "font.size": FONT_SIZE_PT,
            "axes.labelsize": FONT_SIZE_PT,
            "axes.titlesize": FONT_SIZE_PT,
            "xtick.labelsize": FONT_SIZE_PT,
            "ytick.labelsize": FONT_SIZE_PT,
            "legend.fontsize": FONT_SIZE_PT,
            "axes.facecolor": "white",
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
            "text.usetex": True,
            "text.latex.preamble": "\n".join(
                (
                    r"\usepackage{newtxtext}",
                    r"\usepackage{newtxmath}",
                    r"\usepackage{siunitx}",
                    r"\sisetup{detect-weight=true, detect-family=true}",
                    r"\sisetup{per-mode=symbol}",
                    r"\sisetup{inter-unit-product = \mathord{\cdot}}",
                )
            ),
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "path",
        }
    )


def build_violin_figure(series_list: tuple[SceneMetricSeries, ...], plt):
    configure_plot_style(plt)

    values_by_scene = [series.values for series in series_list]
    labels = [series.spec.scene_label for series in series_list]
    positions = np.arange(1, len(series_list) + 1)

    fig, ax = plt.subplots(figsize=(FIGURE_WIDTH_INCH, FIGURE_HEIGHT_INCH))
    fig.subplots_adjust(
        left=AXES_LEFT,
        right=AXES_RIGHT,
        bottom=AXES_BOTTOM,
        top=AXES_TOP,
    )

    violin = ax.violinplot(
        values_by_scene,
        positions=positions,
        widths=VIOLIN_WIDTH,
        showmeans=False,
        showmedians=False,
        showextrema=False,
        bw_method="scott",
    )
    for body, color in zip(violin["bodies"], SCENE_COLORS):
        body.set_facecolor(color)
        body.set_edgecolor("black")
        body.set_alpha(VIOLIN_ALPHA)
        body.set_linewidth(0.65)

    ax.axhline(
        FC_RP_MIN_THRESHOLD,
        color=THRESHOLD_COLOR,
        linestyle="--",
        linewidth=0.95,
        alpha=0.95,
        label=THRESHOLD_LABEL,
        zorder=2,
    )

    for position, values in zip(positions, values_by_scene):
        quartile_1, median, quartile_3 = np.percentile(values, [25, 50, 75])
        ax.vlines(
            position,
            quartile_1,
            quartile_3,
            color="black",
            linewidth=QUARTILE_LINE_WIDTH,
            zorder=4,
        )
        ax.hlines(
            median,
            position - 0.18,
            position + 0.18,
            color="black",
            linewidth=MEDIAN_LINE_WIDTH,
            zorder=5,
        )

    all_values = [value for values in values_by_scene for value in values]
    all_values.append(FC_RP_MIN_THRESHOLD)
    y_min = min(all_values)
    y_max = max(all_values)
    y_span = y_max - y_min
    if y_span <= 0.0:
        y_span = max(abs(y_max), 1.0)
    ax.set_ylim(y_min - 0.08 * y_span, y_max + 0.08 * y_span)

    ax.set_xlim(0.45, len(series_list) + 0.55)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels)
    ax.set_ylabel(Y_LABEL, labelpad=1.0)
    ax.tick_params(axis="both", which="major", pad=1.5, length=2.4, width=0.6)
    ax.legend(
        loc="upper right",
        frameon=True,
        framealpha=0.88,
        edgecolor="0.75",
        borderpad=0.2,
        handlelength=1.2,
        handletextpad=0.35,
        labelspacing=0.2,
    )
    ax.grid(True, axis="y", which="major", color="0.82", linewidth=0.45, alpha=0.7)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_linewidth(0.65)

    return fig


def print_summary(series_list: tuple[SceneMetricSeries, ...]) -> None:
    for series in series_list:
        values = series.values
        print(
            "{}: n={}, min={:.6f}, mean={:.6f}, median={:.6f}, max={:.6f}".format(
                series.spec.scene_label,
                len(values),
                min(values),
                statistics.fmean(values),
                statistics.median(values),
                max(values),
            ),
            flush=True,
        )


def main() -> int:
    roscore_process: subprocess.Popen | None = None
    try:
        bag_specs = tuple(BagSpec(label, Path(path)) for label, path in BAG_SPECS)

        roscore_process = start_roscore_if_needed()
        ensure_clean_ros_graph()
        set_use_sim_time()
        initialize_rospy_node()

        series_list = tuple(replay_and_collect(spec) for spec in bag_specs)
        print_summary(series_list)

        write_samples_csv(series_list, OUTPUT_CSV_PATH)
        print("Saved samples: {}".format(OUTPUT_CSV_PATH.resolve()), flush=True)

        plt = import_pyplot()
        fig = build_violin_figure(series_list, plt)
        OUTPUT_SVG_PATH.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(
            OUTPUT_SVG_PATH,
            dpi=SAVE_DPI,
            format="svg",
            bbox_inches="tight",
            pad_inches=0.0,
        )
        plt.close(fig)
        print("Saved plot: {}".format(OUTPUT_SVG_PATH.resolve()), flush=True)

        return 0
    except (RuntimeError, rosbag.ROSBagException) as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        stop_process_group(roscore_process)


if __name__ == "__main__":
    sys.exit(main())
