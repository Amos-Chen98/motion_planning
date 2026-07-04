#!/usr/bin/env python3

import argparse
import datetime
import os
import shlex
import subprocess
import sys


PACKAGE_NAME = "multilink_copilot"


def split_rosbag_args(argv):
    if "--" not in argv:
        return argv, []

    separator_index = argv.index("--")
    return argv[:separator_index], argv[separator_index + 1 :]


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Select a multilink_copilot rosbag and replay it with RViz visualization.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("bag", nargs="?", help="Bag path, or a bag filename under data_manager/dragon_copilot/data/rosbag.")
    parser.add_argument("--latest", action="store_true", help="Replay the newest bag without prompting.")
    parser.add_argument("--rate", type=float, help="Playback rate passed to rosbag play.")
    parser.add_argument("--pause", action="store_true", help="Start rosbag play paused.")
    parser.add_argument("--loop", action="store_true", help="Loop rosbag playback.")
    parser.add_argument("--no-rviz", action="store_true", help="Do not launch RViz.")
    parser.add_argument("--list", action="store_true", help="List available bags and exit.")

    roslaunch_args, extra_play_args = split_rosbag_args(argv)
    args = parser.parse_args(roslaunch_args)
    args.extra_play_args = extra_play_args

    if args.latest and args.bag:
        parser.error("--latest cannot be used together with an explicit bag path.")

    return args


def find_package_path(package_name):
    try:
        output = subprocess.check_output(["rospack", "find", package_name], text=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError("failed to locate {} with rospack: {}".format(package_name, exc))
    return output.strip()


def list_bags(rosbag_dir):
    if not os.path.isdir(rosbag_dir):
        return []

    bags = [
        os.path.join(rosbag_dir, filename)
        for filename in os.listdir(rosbag_dir)
        if filename.endswith(".bag") and os.path.isfile(os.path.join(rosbag_dir, filename))
    ]
    return sorted(bags, key=lambda path: (os.path.getmtime(path), os.path.basename(path)), reverse=True)


def format_size(byte_count):
    units = ["B", "KB", "MB", "GB", "TB"]
    size = float(byte_count)
    for unit in units:
        if size < 1024.0 or unit == units[-1]:
            return "{:.1f} {}".format(size, unit)
        size /= 1024.0


def print_bags(bags):
    if not bags:
        print("No .bag files found.")
        return

    print("Available rosbag files:")
    for index, path in enumerate(bags, start=1):
        modified = datetime.datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y-%m-%d %H:%M:%S")
        print(
            "{:>2}. {}  {}  {}".format(
                index,
                os.path.basename(path),
                format_size(os.path.getsize(path)),
                modified,
            )
        )


def choose_bag_interactively(bags):
    print_bags(bags)
    print("")

    while True:
        selection = input("Select bag index (q to quit): ").strip()
        if selection.lower() in ("q", "quit", "exit"):
            return None

        try:
            index = int(selection)
        except ValueError:
            print("Invalid selection: enter a number from 1 to {}.".format(len(bags)))
            continue

        if 1 <= index <= len(bags):
            return bags[index - 1]

        print("Invalid selection: enter a number from 1 to {}.".format(len(bags)))


def resolve_bag_path(bag_arg, rosbag_dir):
    if not bag_arg:
        return None

    candidates = [bag_arg]
    if not os.path.isabs(bag_arg):
        candidates.append(os.path.join(os.getcwd(), bag_arg))
        candidates.append(os.path.join(rosbag_dir, bag_arg))

    for candidate in candidates:
        candidate = os.path.abspath(os.path.expanduser(candidate))
        if os.path.isfile(candidate):
            return candidate

    raise RuntimeError("bag file does not exist: {}".format(bag_arg))


def build_play_args(args):
    play_args = ["--clock"]

    if args.pause:
        play_args.append("--pause")
    if args.loop:
        play_args.append("--loop")
    if args.rate is not None:
        if args.rate <= 0.0:
            raise RuntimeError("--rate must be greater than 0.")
        play_args.extend(["--rate", "{:g}".format(args.rate)])

    play_args.extend(args.extra_play_args)
    return play_args


def launch_replay(bag_path, play_args, launch_rviz):
    command = [
        "roslaunch",
        PACKAGE_NAME,
        "replay_rosbag.launch",
        "bag:={}".format(bag_path),
        "rviz:={}".format("false" if not launch_rviz else "true"),
        "play_args:={}".format(shlex.join(play_args)),
    ]

    print("Replaying: {}".format(bag_path))
    print("Command: {}".format(shlex.join(command)))
    return subprocess.call(command)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)

    try:
        data_manager_path = find_package_path("data_manager")
        rosbag_dir = os.path.join(data_manager_path, "dragon_copilot", "data", "rosbag")
        bags = list_bags(rosbag_dir)

        if args.list:
            print_bags(bags)
            return 0

        if not bags and not args.bag:
            raise RuntimeError("no .bag files found in {}".format(rosbag_dir))

        bag_path = resolve_bag_path(args.bag, rosbag_dir)
        if bag_path is None:
            if args.latest:
                bag_path = bags[0]
            else:
                if not sys.stdin.isatty():
                    raise RuntimeError("no bag specified and stdin is not interactive")
                bag_path = choose_bag_interactively(bags)
                if bag_path is None:
                    return 1

        return launch_replay(bag_path, build_play_args(args), not args.no_rviz)
    except RuntimeError as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
