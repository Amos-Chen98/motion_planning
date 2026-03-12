# anchor_navigation

Hierarchical Trajectory Planning of Floating-Base Multi-Link Robot for Maneuvering in Confined Environments

Part of the [motion_planning](https://github.com/ut-dragon-lab/motion_planning) meta-package, located under `aerial_motion/`.

## 1 Installation

### Dependencies

Follow the instructions in [motion_planning README](../../README.md).

In addition,

```bash
sudo apt update
sudo apt install libnlopt-dev python3-nlopt
sudo apt install ros-$ROS_DISTRO-octomap*
# if using ros-one, the above command should be sudo apt install ros-one-octomap*
pip install pyquaternion

```

Install Pinocchio:

**(a) On your PC:**

```bash
pip install pin
```

**(b) On the robot's onboard computer**, build from source following the instructions at:

https://stack-of-tasks.github.io/pinocchio/download.html

```bash
cd <your_motion_planning_ws>
wstool merge -t src src/motion_planning/noetic.rosinstall
wstool update -t src
```

### Build

```bash
cd <your_motion_planning_ws>
catkin build
```

## 2 Usage

### 2.1 Planning and tracking

Both modes share the same bringup step:

```bash
# Launch robot model in Gazebo, RViz, and publish point cloud
roslaunch anchor_navigation bringup_urdf.launch

# Takeoff the robot
rosrun aerial_robot_base keyboard_command.py
```

Then choose one of the following modes:

#### 2.1.1 Default target: all joint angles set to +90 degrees

```bash
# Launch motion planner and map server
roslaunch anchor_navigation motion_planner.launch

# Set target root link pose using 2D Nav Goal in RViz
```

#### 2.1.2 Arbitrary target state

```bash
# Launch motion planner and map server
roslaunch anchor_navigation motion_planner.launch is_goal_complete:=true

# Publish the desired target state via topic
rostopic pub -1 /hydrus_xi/target_state motion_planner/HydrusConfig "rootlink_pos_x: -0.8
rootlink_pos_y: 1.4
rootlink_yaw: 3.0
joint_angles: [1.0, 1.2, 1.2]"
```

### 2.2 Replay saved trajectory

```bash
roslaunch anchor_navigation bringup_urdf.launch
roslaunch anchor_navigation traj_replay.launch
```

### 2.3 Replay a rosbag with customized RViz config

```bash
roslaunch anchor_navigation review_data.launch
rosbag play <bag_name.bag>
```

### 2.4 Batch testing

See [bash/README.md](bash/README.md) for details.

### 2.5 Review recorded data

See [scripts/data_review/README.md](scripts/data_review/README.md) for details.