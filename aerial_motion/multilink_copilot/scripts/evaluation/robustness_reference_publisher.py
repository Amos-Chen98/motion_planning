#!/usr/bin/env python3
"""Publish the existing nonholonomic trajectory, with explicit trial boundaries."""
import argparse
import json
from pathlib import Path
import sys
import time

import rospkg
import rospy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


def build_samples(case_file, duration, rate):
    # Keep this public entry point for the robustness runner and visualization.
    script_dir = Path(rospkg.RosPack().get_path("mono_planner")) / "scripts"
    if str(script_dir) not in sys.path:
        sys.path.insert(0, str(script_dir))
    from traj_planner.waypoint_reference import build_samples as generate
    return generate(case_file, duration, rate)

def build_visualization(case_file, samples, duration, rate):
    """Display the exact tail-reference samples and all case waypoints in world coordinates."""
    from traj_planner.waypoint_planner_common import interpolate_time_color
    from demo.waypoint_ring_visualizer import WaypointRingVisualizer

    data = yaml.safe_load(Path(case_file).read_text())
    poses, rings = [], MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
    for index, waypoint in enumerate(data["waypoints"]):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position = Point(*waypoint[:3])
        q = quaternion_from_euler(*waypoint[3:])
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        poses.append(pose)
        ring = Marker()
        ring.header.frame_id = "world"
        ring.ns, ring.id = "waypoint_rings", index
        ring.type, ring.action = Marker.LINE_STRIP, Marker.ADD
        ring.pose = pose.pose
        ring.scale.x = WaypointRingVisualizer.RING_THICKNESS_METERS
        ring.color.b = ring.color.a = 1.0
        ring.points = WaypointRingVisualizer._build_ring_points()
        rings.markers.append(ring)

    line = Marker()
    line.header.frame_id = "world"
    line.ns, line.id = "trajectory_line", 0
    line.type, line.action = Marker.LINE_STRIP, Marker.ADD
    line.pose.orientation.w = 1.0
    line.scale.x = 0.05
    line.color.r = line.color.g = line.color.b = line.color.a = 1.0
    line.points = [msg.pose.position for msg in samples]
    line.colors = [interpolate_time_color(min(i/rate, duration)/duration) for i in range(len(samples))]
    return poses, rings, MarkerArray(markers=[Marker(action=Marker.DELETEALL), line])


def publish_visualization(case_file, samples, duration, rate):
    poses, rings, trajectory = build_visualization(case_file, samples, duration, rate)
    pose_pubs = [rospy.Publisher("/waypoint/pose_{}".format(i), PoseStamped, queue_size=1, latch=True)
                 for i in range(len(poses))]
    ring_pub = rospy.Publisher("/waypoint/ring_markers", MarkerArray, queue_size=1, latch=True)
    trajectory_pub = rospy.Publisher("/dragon/mono_planner/traj_marker", MarkerArray, queue_size=1, latch=True)
    refresh = rospy.Rate(2)
    while not rospy.is_shutdown():
        for publisher, pose in zip(pose_pubs, poses):
            pose.header.stamp = rospy.Time.now()
            publisher.publish(pose)
        # Static world-frame markers use zero stamps and unlimited lifetimes.
        ring_pub.publish(rings)
        trajectory_pub.publish(trajectory)
        refresh.sleep()
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", required=True)
    parser.add_argument("--duration", type=float, default=50.0)
    parser.add_argument("--rate", type=float, default=40.0)
    parser.add_argument("--visualization-only", action="store_true",
                        help="Publish latched waypoint and time-colored reference markers without flight commands")
    args = parser.parse_args(rospy.myargv()[1:])
    if args.duration <= 0 or args.rate <= 0:
        parser.error("duration and rate must be positive")
    samples = build_samples(args.case, args.duration, args.rate)
    rospy.init_node("robustness_visualization" if args.visualization_only else "robustness_reference")
    if args.visualization_only:
        return publish_visualization(args.case, samples, args.duration, args.rate)
    target = rospy.Publisher("/dragon/root/target_pose", PoseStamped, queue_size=2000)
    events = rospy.Publisher("/dragon/robustness/trial_event", String, queue_size=10, latch=True)
    deadline = time.monotonic() + 30.0
    while target.get_num_connections() < 2 or events.get_num_connections() < 2:
        if time.monotonic() > deadline or rospy.is_shutdown():
            raise RuntimeError("Copilot, recorder and trial monitor must connect before reference playback")
        time.sleep(0.05)
    def event(phase, count):
        events.publish(String(data=json.dumps(dict(
            phase=phase, case=Path(args.case).stem, stamp=rospy.Time.now().to_sec(),
            samples=count, expected_samples=len(samples), duration=args.duration, rate=args.rate))))
    event("started", 0)
    rate = rospy.Rate(args.rate)
    for index, msg in enumerate(samples):
        if rospy.is_shutdown():
            return 130
        msg.header.stamp = rospy.Time.now()
        target.publish(msg)
        if index + 1 < len(samples):
            rate.sleep()
    event("finished", len(samples))
    time.sleep(1.0)  # Allow final queued messages to reach the recorder.
    return 0


if __name__ == "__main__":
    sys.exit(main())
