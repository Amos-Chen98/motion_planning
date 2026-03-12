#!/bin/bash

# Script to run hydrus planning simulation with specified parameters
# Usage: ./run_simulation.sh [world spawn_x spawn_y spawn_yaw target_pos_x target_pos_y target_yaw is_goal_complete find_anchor is_parallel run_traj_opt]
# If no arguments are provided, default parameters will be used

# Check if correct number of arguments provided, use defaults if none
if [ $# -eq 0 ]; then
    echo "No arguments provided, using default parameters:"
    # Default parameters
    WORLD="gap"
    SPAWN_X=2.00
    SPAWN_Y=0
    SPAWN_YAW=2.6
    TARGET_POS_X=-0.8
    TARGET_POS_Y=1.2
    TARGET_YAW=3.141
    IS_GOAL_COMPLETE="false"
    IS_MULTI_SHOOTING="true"
    IS_PARALLEL="true"
    RUN_TRAJ_OPT="true"
elif [ $# -ne 11 ]; then
    echo "Usage: $0 [world spawn_x spawn_y spawn_yaw target_pos_x target_pos_y target_yaw is_goal_complete find_anchor is_parallel run_traj_opt]"
    echo "Example: $0 gap 0.0 0.0 0.0 5.0 3.0 1.57 true false true true"
    echo "Or run without arguments to use default parameters"
    exit 1
else
    # Parse command line arguments
    WORLD=$1
    SPAWN_X=$2
    SPAWN_Y=$3
    SPAWN_YAW=$4
    TARGET_POS_X=$5
    TARGET_POS_Y=$6
    TARGET_YAW=$7
    IS_GOAL_COMPLETE=$8
    IS_MULTI_SHOOTING=$9
    IS_PARALLEL=${10}
    RUN_TRAJ_OPT=${11}
fi

echo "Starting simulation with parameters:"
echo "World: $WORLD"
echo "Spawn position: ($SPAWN_X, $SPAWN_Y) with yaw: $SPAWN_YAW"
echo "Target position: ($TARGET_POS_X, $TARGET_POS_Y) with yaw: $TARGET_YAW"
echo "Goal complete: $IS_GOAL_COMPLETE, Multi-shooting: $IS_MULTI_SHOOTING, Parallel: $IS_PARALLEL, Run trajectory optimization: $RUN_TRAJ_OPT"

# Function to calculate quaternion from yaw angle
calculate_quaternion() {
    local yaw=$1
    # For rotation around z-axis: qz = sin(yaw/2), qw = cos(yaw/2)
    local qz=$(echo "scale=6; s($yaw/2)" | bc -l)
    local qw=$(echo "scale=6; c($yaw/2)" | bc -l)
    echo "$qz $qw"
}

# Function to cleanup all processes
cleanup() {
    echo "Shutting down all processes..."
    
    # First, try graceful shutdown of ROS nodes
    echo "Attempting graceful shutdown of ROS nodes..."
    rosnode kill -a 2>/dev/null
    sleep 2
    
    # Then kill processes with SIGTERM (allows cleanup handlers)
    echo "Sending SIGTERM to processes..."
    pkill -TERM -f "roslaunch motion_planner motion_planner.launch" 2>/dev/null
    pkill -TERM -f "roslaunch motion_planner bringup_urdf.launch" 2>/dev/null
    pkill -TERM -f "keyboard_command.py" 2>/dev/null
    pkill -TERM -f "rostopic pub" 2>/dev/null
    pkill -TERM -f "send_robot_commands.py" 2>/dev/null
    
    # Wait for processes to terminate gracefully
    echo "Waiting for graceful termination..."
    sleep 3
    
    # Force kill any remaining processes
    echo "Force killing any remaining processes..."
    pkill -9 -f "roslaunch motion_planner bringup_urdf.launch" 2>/dev/null
    pkill -9 -f "keyboard_command.py" 2>/dev/null
    pkill -9 -f "roslaunch motion_planner motion_planner.launch" 2>/dev/null
    pkill -9 -f "rostopic pub" 2>/dev/null
    pkill -9 -f "send_robot_commands.py" 2>/dev/null
    
    # Kill roscore if running
    pkill -f roscore 2>/dev/null
    
    # Clean up temporary files
    rm -f /tmp/send_robot_commands.py
    rm -f /tmp/publish_goal.py

    # Close all gnome-terminal windows that were opened by this script
    echo "Closing all terminal tabs..."
    pkill -f "gnome-terminal" 2>/dev/null

    echo "All processes and terminals closed."
    exit 0
}

# Set up signal handler for cleanup
trap cleanup SIGINT SIGTERM

echo "Step 1: Launching bringup_urdf.launch..."
# Launch bringup_urdf.launch in a new terminal tab
gnome-terminal --tab --title="Bringup URDF" -- bash -c "
    roslaunch motion_planner bringup_urdf.launch world_name:=$WORLD spawn_x:=$SPAWN_X spawn_y:=$SPAWN_Y spawn_yaw:=$SPAWN_YAW
    exec bash
" &

# Wait for 8 seconds
echo "Waiting 8 seconds for system initialization..."
sleep 8

echo "Step 2: Starting robot command sequence..."

gnome-terminal --tab --title="keyboard" -- bash -lc "expect <<'EOF'
spawn rosrun aerial_robot_base keyboard_command.py
sleep 2
send \"r\r\"
sleep 1
send \"t\r\"
sleep 1
exit
EOF
"

echo "Step 3: Launching motion planner..."
# Launch motion_planner.launch in another tab
gnome-terminal --tab --title="Motion Planner" -- bash -c "
    roslaunch motion_planner motion_planner.launch is_goal_complete:=$IS_GOAL_COMPLETE find_anchor:=$IS_MULTI_SHOOTING is_parallel:=$IS_PARALLEL run_traj_opt:=$RUN_TRAJ_OPT
    exec bash
" &

# Wait for 30 seconds
echo "Waiting 30 seconds for robot to take off..."
sleep 30

echo "Step 4: Publishing target goal..."
# Calculate quaternion values for target yaw
read QZ QW <<<$(calculate_quaternion $TARGET_YAW)

echo "Publishing goal with quaternion: qz=$QZ, qw=$QW"

# Create a small Python publisher to publish PoseStamped with proper float types
cat >/tmp/publish_goal.py <<EOF
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('goal_publisher', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    # Give ROS time to start up and for connections to establish
    rospy.sleep(1.0)

    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = float($TARGET_POS_X)
    msg.pose.position.y = float($TARGET_POS_Y)
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = float($QZ)
    msg.pose.orientation.w = float($QW)

    pub.publish(msg)
    rospy.loginfo('Goal published successfully')

if __name__ == '__main__':
    main()
EOF

# Make the script executable and run it in a new terminal tab
chmod +x /tmp/publish_goal.py
gnome-terminal --tab --title="Goal Publisher" -- bash -c "
    python3 /tmp/publish_goal.py
    exec bash
" &

echo "Goal published. Waiting 120 s before shutdown..."
sleep 120

echo "Simulation completed. Closing all terminals and cleaning up..."
# Cleanup and shutdown
cleanup