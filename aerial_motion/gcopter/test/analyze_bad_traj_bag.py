#!/usr/bin/env python3
"""Analyze gcopter/PolyTraj trajectories recorded in a rosbag.

Evaluates every piecewise-quintic trajectory on a trajectory topic and
reports its spatial extent, flagging trajectories that leave the planning
volume (map_bound). Optionally dumps the planner's /rosout log lines so
optimization failures and time-scaling events can be correlated with the
anomalous trajectories.

Written for the 2026-06-23 Naraha flight bag
(2026-06-23-15-11-32_quad_bad_traj.bag); see
gcopter/test/README_GCOPTER_BAD_TRAJ.md for the full analysis.

Example:
    rosrun gcopter analyze_bad_traj_bag.py \
        path/to/2026-06-23-15-11-32_quad_bad_traj.bag
"""

import argparse

import numpy as np
import rosbag


def evaluate_piece(coef_x, coef_y, coef_z, ts, derivative=0):
    """Evaluate one polynomial piece (or its derivative) at times ts.

    Coefficients follow the gcopter Piece<5> convention: column 0 multiplies
    t^5 and column 5 is the constant term (see trajectory.hpp getPos()).
    """
    coeffs = np.vstack([coef_x, coef_y, coef_z])
    for _ in range(derivative):
        coeffs = coeffs[:, :-1] * np.arange(coeffs.shape[1] - 1, 0, -1)
    degree = coeffs.shape[1] - 1
    powers = np.vstack([ts ** (degree - i) for i in range(degree + 1)])
    return coeffs @ powers


def evaluate_trajectory(msg, samples_per_piece):
    positions = []
    velocities = []
    for piece in range(len(msg.durations)):
        s = 6 * piece
        coef = (msg.coef_x[s:s + 6], msg.coef_y[s:s + 6], msg.coef_z[s:s + 6])
        ts = np.linspace(0.0, msg.durations[piece], samples_per_piece)
        positions.append(evaluate_piece(*coef, ts))
        velocities.append(evaluate_piece(*coef, ts, derivative=1))
    return np.hstack(positions), np.hstack(velocities)


def out_of_bound_axes(positions, bound, margin):
    lo = np.array(bound[0::2]) - margin
    hi = np.array(bound[1::2]) + margin
    flags = []
    for axis, name in enumerate("xyz"):
        if positions[axis].min() < lo[axis] or positions[axis].max() > hi[axis]:
            flags.append(name)
    return flags


def dump_planner_log(bag_path, t0):
    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=["/rosout"]):
            if "planner" in msg.name or "gcopter" in msg.name:
                print("%7.2f [%s] %s" % (t.to_sec() - t0, msg.name, msg.msg))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="rosbag containing gcopter/PolyTraj messages")
    parser.add_argument("--topic", default="/planning/trajectory",
                        help="trajectory topic (default: %(default)s)")
    parser.add_argument("--map-bound", type=float, nargs=6,
                        default=[-8.0, 8.0, -8.0, 8.0, -1.0, 6.0],
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
                        help="planning volume used to flag anomalies "
                             "(default: %(default)s)")
    parser.add_argument("--margin", type=float, default=0.2,
                        help="tolerated overshoot beyond map_bound in meters, "
                             "absorbing normal soft-penalty violations "
                             "(default: %(default)s)")
    parser.add_argument("--samples-per-piece", type=int, default=100)
    parser.add_argument("--show-log", action="store_true",
                        help="also dump the planner's /rosout messages")
    args = parser.parse_args()

    header = ("%3s %7s %4s %4s %9s "
              "%8s %8s %8s %8s %8s %8s %7s  %s" %
              ("#", "t[s]", "id", "pcs", "dur[s]",
               "x_min", "x_max", "y_min", "y_max", "z_min", "z_max",
               "v_max", "flags"))
    print(header)

    t0 = None
    all_positions = []
    anomalous = []
    count = 0
    with rosbag.Bag(args.bag) as bag:
        for _, msg, t in bag.read_messages(topics=[args.topic]):
            if t0 is None:
                t0 = t.to_sec()
            positions, velocities = evaluate_trajectory(
                msg, args.samples_per_piece)
            all_positions.append(positions)
            count += 1
            flags = out_of_bound_axes(positions, args.map_bound, args.margin)
            if flags:
                anomalous.append(msg.traj_id)
            print("%3d %7.2f %4d %4d %9.2f "
                  "%8.2f %8.2f %8.2f %8.2f %8.2f %8.2f %7.2f  %s" %
                  (count, t.to_sec() - t0, msg.traj_id, len(msg.durations),
                   float(np.sum(msg.durations)),
                   positions[0].min(), positions[0].max(),
                   positions[1].min(), positions[1].max(),
                   positions[2].min(), positions[2].max(),
                   float(np.linalg.norm(velocities, axis=0).max()),
                   "OUT-OF-BOUND(%s)" % ",".join(flags) if flags else ""))

    if not all_positions:
        print("No messages found on %s" % args.topic)
        return

    merged = np.hstack(all_positions)
    print("\n%d trajectories, %d beyond map_bound %s (margin %.2f m): ids %s" %
          (count, len(anomalous), args.map_bound, args.margin, anomalous))
    print("Overall extent: x [%.2f, %.2f], y [%.2f, %.2f], z [%.2f, %.2f]" %
          (merged[0].min(), merged[0].max(),
           merged[1].min(), merged[1].max(),
           merged[2].min(), merged[2].max()))

    if args.show_log:
        print("\nPlanner log (/rosout):")
        dump_planner_log(args.bag, t0)


if __name__ == "__main__":
    main()
