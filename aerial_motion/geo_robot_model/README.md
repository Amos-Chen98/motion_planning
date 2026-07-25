# geo_robot_model

`geo_robot_model` provides lightweight, Gazebo-free geometric robot models and simulated LiDAR sensing for aerial motion-planning tests. The models update their state directly from command messages, so they are intended for planner integration and visualization rather than flight-dynamics evaluation.

## Components

- `point_robot_model.py`: ideal point-robot model that follows `aerial_robot_msgs/FlightNav` commands and publishes odometry and TF.
- `geo_dragon_model.py`: ideal four-link DRAGON model that publishes its geometric state, complete URDF joint state, root/CoG TF, and optional lightweight visualization from root and joint commands.
- `livox_mid360_simulator`: converts a global point-cloud map into a local Livox Mid-360 point cloud using robot odometry and the robot-provided LiDAR TF.
- `pose_to_flight_nav.py`: converts `geometry_msgs/PoseStamped` commands to `aerial_robot_msgs/FlightNav`.

## Build

```bash
catkin build geo_robot_model
source devel/setup.bash
```

## Run

Start the point-robot model:

```bash
roslaunch geo_robot_model point_robot_bringup.launch
```

Start the geometric DRAGON model:

```bash
roslaunch geo_robot_model geo_dragon_bringup.launch
```

The geometric DRAGON bringup publishes `/dragon/robot_description` from the canonical `dragon/robots/quad/v1_5_202601.urdf.xacro` model and starts `robot_state_publisher`. The geometric node publishes only the moving `world -> root` transform and the independent `world -> cog` transform; `robot_state_publisher` exclusively owns the articulated link and sensor TF tree. RViz therefore displays the same v1.5 link, battery, gimbal, and Livox meshes as the standard DRAGON simulation while all state updates remain lightweight and Gazebo-free. The cylinder-and-sphere geometric proxy is still published on `/dragon/robot_markers` for debugging but is disabled in the default RViz configuration.

`/dragon/joint_states` contains the six commanded inter-link joints followed by the eight gimbal joints and four rotor joints required to reconstruct the complete URDF tree. The geometric model keeps the gimbal and rotor positions at zero; planners continue to select the six inter-link joints by name.

To debug the Copilot planner, start the geometric model and planner in separate terminals. Wait until the first launch reports that the DRAGON geometric model is ready before starting Copilot, because the planner reads `robot_description` during initialization.

Override `robot_model` when a different compatible DRAGON xacro is required. When changing `robot_ns`, pass the same value to both launches so that the generated `/<robot_ns>/robot_description`, joint-state input, and full-state target output remain in the same namespace. The bundled RViz RobotModel display is configured for the default `dragon` namespace; use a matching RViz configuration for another namespace.

Pass `lidar:=true` to either bringup launch when a global point-cloud map is available and local simulated LiDAR sensing is required.
