#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
output_dir="$script_dir/../data_analysis/data/rosbag"

mkdir -p "$output_dir"

if [ $# -gt 0 ]; then
  robot_ns="$1"
  shift
else
  robot_ns="dragon"
fi

topics=(
  "$robot_ns/baro"
  "$robot_ns/battery_voltage_status"
  "$robot_ns/debug/four_axes/gain"
  "$robot_ns/debug/pose/pid"
  "$robot_ns/desire_coordinate"
  "$robot_ns/flight_config_ack"
  "$robot_ns/flight_config_cmd"
  "$robot_ns/flight_state"
  "$robot_ns/four_axes/command"
  "$robot_ns/full_state_target"
  "$robot_ns/ground_truth"
  "$robot_ns/imu"
  "$robot_ns/joint_states"
  "$robot_ns/joints_ctrl"
  "$robot_ns/joy"
  "$robot_ns/last_link/tail_pose"
  "$robot_ns/mocap/ground_pose"
  "$robot_ns/mocap/pose"
  "$robot_ns/motor_info"
  "$robot_ns/motor_pwms"
  "$robot_ns/root/pose"
  "$robot_ns/root/target_pose"
  "$robot_ns/root/tail_pose"
  "$robot_ns/rpy/gain"
  "$robot_ns/rpy/pid"
  "$robot_ns/servo/states"
  "$robot_ns/servo/target_states"
  "$robot_ns/servo/torque_enable"
  "$robot_ns/target_pose"
  "$robot_ns/target_rotation_motion"
  "$robot_ns/trajectory_visualization"
  "$robot_ns/uav/baselink/odom"
  "$robot_ns/uav/cog/odom"
  "$robot_ns/uav/full_state"
  "$robot_ns/uav/nav"
  "$robot_ns/uav_info"
  "$robot_ns/uav_power"
  "/waypoint/pose_0"
  "/waypoint/pose_1"
  "/waypoint/pose_2"
  "/waypoint/pose_3"
  "/waypoint/ring_markers"
  "$robot_ns/mono_planner/traj_marker"
  "/tf"
  "/tf_static"
  "rosout"
  "rosout_agg"
)

cd "$output_dir"

exec rosbag record "${topics[@]}" "$@"
