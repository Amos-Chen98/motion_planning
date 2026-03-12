#!/bin/bash

# Script to test planner behavior when point cloud changes
# This script will switch between different world point clouds during execution
# Usage: ./test_world_switching.sh [spawn_x spawn_y spawn_yaw target_pos_x target_pos_y target_yaw is_goal_complete find_anchor is_parallel run_traj_opt]
# If no arguments are provided, default parameters will be used

# Hardcoded world names for testing
# WORLDS=("/large/20251227_4_gap_move" "/large/20251227_5_chair1")
WORLDS=("/large/20251227_4_gap_move" "/large/20251227_2_gap_big")

# Check if correct number of arguments provided, use defaults if none
if [ $# -eq 0 ]; then
    echo "No arguments provided, using default parameters:"
    # Default parameters
    SPAWN_X=-3.5
    SPAWN_Y=0.5
    SPAWN_YAW=0.0
    TARGET_POS_X=1.0
    TARGET_POS_Y=1.0
    TARGET_YAW=0.0
    IS_GOAL_COMPLETE="false"
    IS_MULTI_SHOOTING="true"
    IS_PARALLEL="true"
    RUN_TRAJ_OPT="true"
elif [ $# -ne 10 ]; then
    echo "Usage: $0 [spawn_x spawn_y spawn_yaw target_pos_x target_pos_y target_yaw is_goal_complete find_anchor is_parallel run_traj_opt]"
    echo "Example: $0 0.0 0.0 0.0 5.0 3.0 1.57 true false true true"
    echo "Or run without arguments to use default parameters"
    exit 1
else
    # Parse command line arguments
    SPAWN_X=$1
    SPAWN_Y=$2
    SPAWN_YAW=$3
    TARGET_POS_X=$4
    TARGET_POS_Y=$5
    TARGET_YAW=$6
    IS_GOAL_COMPLETE=$7
    IS_MULTI_SHOOTING=$8
    IS_PARALLEL=$9
    RUN_TRAJ_OPT=${10}
fi

echo "Starting world switching test with parameters:"
echo "Spawn position: ($SPAWN_X, $SPAWN_Y) with yaw: $SPAWN_YAW"
echo "Target position: ($TARGET_POS_X, $TARGET_POS_Y) with yaw: $TARGET_YAW"
echo "Goal complete: $IS_GOAL_COMPLETE, Multi-shooting: $IS_MULTI_SHOOTING, Parallel: $IS_PARALLEL, Run trajectory optimization: $RUN_TRAJ_OPT"
echo "Testing with worlds: ${WORLDS[@]}"

# Store PIDs for process management
BRINGUP_PID=""
PLANNER_PID=""
PCL_PID=""

# Function to calculate quaternion from yaw angle
calculate_quaternion() {
    local yaw=$1
    # For rotation around z-axis: qz = sin(yaw/2), qw = cos(yaw/2)
    local qz=$(echo "scale=6; s($yaw/2)" | bc -l)
    local qw=$(echo "scale=6; c($yaw/2)" | bc -l)
    echo "$qz $qw"
}

# Function to publish goal
publish_goal() {
    local target_x=$1
    local target_y=$2
    local qz=$3
    local qw=$4
    
    echo "Publishing goal: pos=($target_x, $target_y), qz=$qz, qw=$qw"
    
    # Create a small Python publisher to publish PoseStamped with proper float types
    cat >/tmp/publish_goal_test.py <<EOF
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
    msg.pose.position.x = float($target_x)
    msg.pose.position.y = float($target_y)
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = float($qz)
    msg.pose.orientation.w = float($qw)

    pub.publish(msg)
    rospy.loginfo('Goal published successfully')

if __name__ == '__main__':
    main()
EOF

    # Make the script executable and run it
    chmod +x /tmp/publish_goal_test.py
    python3 /tmp/publish_goal_test.py
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
    pkill -TERM -f "roslaunch simulator pub_pcl.launch" 2>/dev/null
    pkill -TERM -f "keyboard_command.py" 2>/dev/null
    pkill -TERM -f "rostopic pub" 2>/dev/null
    
    # Wait for processes to terminate gracefully
    echo "Waiting for graceful termination..."
    sleep 3
    
    # Force kill any remaining processes
    echo "Force killing any remaining processes..."
    pkill -9 -f "roslaunch motion_planner bringup_urdf.launch" 2>/dev/null
    pkill -9 -f "roslaunch simulator pub_pcl.launch" 2>/dev/null
    pkill -9 -f "keyboard_command.py" 2>/dev/null
    pkill -9 -f "roslaunch motion_planner motion_planner.launch" 2>/dev/null
    pkill -9 -f "rostopic pub" 2>/dev/null
    
    # Kill roscore if running
    pkill -f roscore 2>/dev/null
    
    # Clean up temporary files
    rm -f /tmp/publish_goal_test.py

    # Close all gnome-terminal windows that were opened by this script
    echo "Closing all terminal tabs..."
    pkill -f "gnome-terminal" 2>/dev/null

    echo "All processes and terminals closed."
    exit 0
}

# Set up signal handler for cleanup
trap cleanup SIGINT SIGTERM

echo "Step 1: Launching bringup_urdf.launch with pub_pcl:=false..."
# Launch bringup_urdf.launch without point cloud publishing
gnome-terminal --tab --title="Bringup URDF" -- bash -c "
    roslaunch motion_planner bringup_urdf.launch world_name:=${WORLDS[0]} spawn_x:=$SPAWN_X spawn_y:=$SPAWN_Y spawn_yaw:=$SPAWN_YAW pub_pcl:=false
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

echo "Step 3: Launching first world point cloud (${WORLDS[0]})..."
# Launch pub_pcl.launch for the first world in a new tab
gnome-terminal --tab --title="PCL Publisher - ${WORLDS[0]}" -- bash -c "
    roslaunch simulator pub_pcl.launch world_name:=${WORLDS[0]}
    exec bash
" &
PCL_PID=$!

echo "Step 4: Launching motion planner..."
# Launch motion_planner.launch in another tab
gnome-terminal --tab --title="Motion Planner" -- bash -c "
    roslaunch motion_planner motion_planner.launch is_goal_complete:=$IS_GOAL_COMPLETE find_anchor:=$IS_MULTI_SHOOTING is_parallel:=$IS_PARALLEL run_traj_opt:=$RUN_TRAJ_OPT
    exec bash
" &

# Wait for 30 seconds
echo "Waiting 30 seconds for robot to take off..."
sleep 30

echo "Step 5: Publishing initial target goal..."
# Calculate quaternion values for target yaw
read QZ QW <<<$(calculate_quaternion $TARGET_YAW)

# Publish the goal for the first time
publish_goal $TARGET_POS_X $TARGET_POS_Y $QZ $QW

echo ""
echo "==========================================="
echo "First goal published with world: ${WORLDS[0]}"
echo "Press any key to switch to the next world..."
echo "==========================================="
read -n 1 -s

# Loop through remaining worlds
for i in "${!WORLDS[@]}"; do
    # Skip the first world since it's already published
    if [ $i -eq 0 ]; then
        continue
    fi
    
    WORLD="${WORLDS[$i]}"
    
    echo ""
    echo "Step $(($i + 5)): Switching to world: $WORLD"
    
    # Kill the current pub_pcl.launch process
    echo "Stopping previous point cloud publisher..."
    pkill -TERM -f "roslaunch simulator pub_pcl.launch" 2>/dev/null
    sleep 1
    pkill -9 -f "roslaunch simulator pub_pcl.launch" 2>/dev/null
    
    # Launch pub_pcl.launch for the next world
    echo "Starting point cloud publisher for $WORLD..."
    gnome-terminal --tab --title="PCL Publisher - $WORLD" -- bash -c "
        roslaunch simulator pub_pcl.launch world_name:=$WORLD
        exec bash
    " &
    
    # Wait 1 second before publishing goal
    echo "Waiting a while before publishing goal..."
    sleep 10
    
    # Publish the same goal again
    echo "Publishing target goal for world: $WORLD"
    publish_goal $TARGET_POS_X $TARGET_POS_Y $QZ $QW
    
    # If not the last world, wait for keyboard input
    if [ $i -lt $((${#WORLDS[@]} - 1)) ]; then
        echo ""
        echo "==========================================="
        echo "Goal published with world: $WORLD"
        echo "Press any key to switch to the next world..."
        echo "==========================================="
        read -n 1 -s
    fi
done

echo ""
echo "==========================================="
echo "All worlds have been tested!"
echo "Final world: ${WORLDS[-1]}"
echo "Simulation will continue running."
echo "Press Ctrl+C to exit and cleanup."
echo "==========================================="

# Keep the script running until Ctrl+C
while true; do
    sleep 1
done
