#!/bin/bash

# Batch test script for hydrus planning simulation
# This script runs test scenarios with multiple configurations each
# Total simulation runs = number of predefined scenarios * NUM_CONFIGS
#
# Features:
# - Progress tracking: Records completed tests to allow resumption after interruption
# - Graceful interruption handling: Press Ctrl+C to stop and save progress
# - Resume capability: Automatically detects incomplete test runs and offers to resume

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
RUN_SIMULATION_SCRIPT="$SCRIPT_DIR/run_simulation.sh"
TEST_CONFIG_FILE="$SCRIPT_DIR/test_config.sh"
PROGRESS_DIR="$SCRIPT_DIR/bash_logs/progress"

# Create progress directory
mkdir -p "$PROGRESS_DIR"

# Load test configuration
if [ ! -f "$TEST_CONFIG_FILE" ]; then
    echo "Error: Test configuration file not found: $TEST_CONFIG_FILE"
    exit 1
fi

source "$TEST_CONFIG_FILE"

# Calculate test parameters
NUM_CONFIGS=${#CONFIGS[@]}
NUM_SCENARIOS=${#PREDEFINED_TEST_SCENARIOS[@]}
declare -a TEST_SCENARIOS=("${PREDEFINED_TEST_SCENARIOS[@]}")

# Check if run_simulation.sh exists
if [ ! -f "$RUN_SIMULATION_SCRIPT" ]; then
    echo "Error: run_simulation.sh not found at $RUN_SIMULATION_SCRIPT"
    exit 1
fi

# Make sure run_simulation.sh is executable
chmod +x "$RUN_SIMULATION_SCRIPT"

# Generate unique session ID
SESSION_ID="batch_test_$(date +%Y%m%d_%H%M%S)"
PROGRESS_FILE="$PROGRESS_DIR/${SESSION_ID}_progress.txt"
INTERRUPTED_FLAG="$PROGRESS_DIR/.interrupted"

# Flag to track if we should stop
STOP_REQUESTED=false

# Trap SIGINT (Ctrl+C) and SIGTERM
trap 'handle_interrupt' SIGINT SIGTERM

# Function to handle interruption
handle_interrupt() {
    echo ""
    echo "========================================="
    echo "Interruption detected!"
    echo "========================================="
    STOP_REQUESTED=true
    touch "$INTERRUPTED_FLAG"
    echo "Progress saved. You can resume this test session later."
    echo "Progress file: $PROGRESS_FILE"
    exit 130
}

# Function to check for incomplete sessions
check_incomplete_sessions() {
    local incomplete_sessions=()
    
    for progress_file in "$PROGRESS_DIR"/*_progress.txt; do
        [ -f "$progress_file" ] || continue
        
        # Extract session info
        local session_name=$(basename "$progress_file" _progress.txt)
        local total_tests=$(grep "^TOTAL_TESTS=" "$progress_file" | cut -d'=' -f2)
        local completed_tests=$(grep -c "^COMPLETED:" "$progress_file")
        
        if [ "$completed_tests" -lt "$total_tests" ]; then
            incomplete_sessions+=("$progress_file|$session_name|$completed_tests|$total_tests")
        fi
    done
    
    if [ ${#incomplete_sessions[@]} -gt 0 ]; then
        echo ""
        echo "========================================="
        echo "Incomplete Test Sessions Found"
        echo "========================================="
        echo ""
        
        for i in "${!incomplete_sessions[@]}"; do
            IFS='|' read -r pfile sname completed total <<< "${incomplete_sessions[$i]}"
            echo "[$((i+1))] Session: $sname"
            echo "    Progress: $completed/$total tests completed"
            echo "    Started: $(grep "^START_TIME=" "$pfile" | cut -d'=' -f2-)"
        done
        
        echo ""
        echo "Options:"
        echo "  [1-${#incomplete_sessions[@]}] Resume a specific session"
        echo "  [n] Start new test session"
        echo "  [q] Quit"
        echo ""
        read -p "Your choice: " choice
        
        if [ "$choice" = "q" ] || [ "$choice" = "Q" ]; then
            echo "Exiting."
            exit 0
        elif [ "$choice" = "n" ] || [ "$choice" = "N" ]; then
            return 0
        elif [ "$choice" -ge 1 ] && [ "$choice" -le "${#incomplete_sessions[@]}" ]; then
            IFS='|' read -r pfile sname completed total <<< "${incomplete_sessions[$((choice-1))]}"
            PROGRESS_FILE="$pfile"
            SESSION_ID="$sname"
            echo "Resuming session: $SESSION_ID"
            return 1
        else
            echo "Invalid choice. Starting new session."
            return 0
        fi
    fi
    
    return 0
}

# Check for incomplete sessions
RESUME_SESSION=false
check_incomplete_sessions
if [ $? -eq 1 ]; then
    RESUME_SESSION=true
fi

# Display test configuration
echo "========================================="
echo "Batch Test Configuration"
echo "========================================="
echo "Session ID: $SESSION_ID"
echo "Scenarios: $NUM_SCENARIOS"
echo "Configurations: $NUM_CONFIGS"
echo "Total tests: $((NUM_SCENARIOS * NUM_CONFIGS))"
echo ""

if [ "$RESUME_SESSION" = false ]; then
    echo "Sample scenarios (first 5):"
    for i in $(seq 0 $((NUM_SCENARIOS > 5 ? 4 : NUM_SCENARIOS-1))); do
        read spawn_x spawn_y spawn_yaw target_pos_x target_pos_y target_yaw <<< "${TEST_SCENARIOS[$i]}"
        echo "  Scenario $((i+1)): spawn=($spawn_x, $spawn_y, $spawn_yaw) target=($target_pos_x, $target_pos_y, $target_yaw)"
    done
    [ $NUM_SCENARIOS -gt 5 ] && echo "  ... and $((NUM_SCENARIOS - 5)) more scenarios"
    echo ""
fi

echo "========================================="
echo ""

# Create bash_logs directory if it doesn't exist
BASH_LOGS_DIR="$SCRIPT_DIR/bash_logs"
mkdir -p "$BASH_LOGS_DIR"

# Create or resume summary log file
if [ "$RESUME_SESSION" = true ]; then
    LOG_FILE="$BASH_LOGS_DIR/${SESSION_ID}_summary.txt"
    echo "" >> "$LOG_FILE"
    echo "=========================================" >> "$LOG_FILE"
    echo "Test Session Resumed: $(date)" >> "$LOG_FILE"
    echo "=========================================" >> "$LOG_FILE"
    echo "" >> "$LOG_FILE"
else
    LOG_FILE="$BASH_LOGS_DIR/${SESSION_ID}_summary.txt"
    
    # Initialize progress file
    {
        echo "SESSION_ID=$SESSION_ID"
        echo "START_TIME=$(date)"
        echo "TOTAL_TESTS=$((NUM_SCENARIOS * NUM_CONFIGS))"
        echo "NUM_SCENARIOS=$NUM_SCENARIOS"
        echo "NUM_CONFIGS=$NUM_CONFIGS"
        echo ""
    } > "$PROGRESS_FILE"
fi

# Function to log messages
log_message() {
    local message="$1"
    echo "$message" | tee -a "$LOG_FILE"
}

# Function to check if test is completed
is_test_completed() {
    local scenario_num=$1
    local config_num=$2
    
    if [ ! -f "$PROGRESS_FILE" ]; then
        return 1
    fi
    
    grep -q "^COMPLETED:scenario_${scenario_num}_config_${config_num}$" "$PROGRESS_FILE"
    return $?
}

# Function to mark test as completed
mark_test_completed() {
    local scenario_num=$1
    local config_num=$2
    
    echo "COMPLETED:scenario_${scenario_num}_config_${config_num}" >> "$PROGRESS_FILE"
}

# Function to run single test
run_single_test() {
    local scenario_num=$1
    local config_num=$2
    local scenario_params=$3
    local config_params=$4
    local config_name=$5
    
    # Check if we should stop
    if [ "$STOP_REQUESTED" = true ]; then
        return 1
    fi
    
    local test_name="scenario_${scenario_num}_config_${config_num}"
    
    # Check if already completed
    if is_test_completed "$scenario_num" "$config_num"; then
        log_message "Skipping Test: $test_name (already completed)"
        return 0
    fi
    
    log_message "Running Test: $test_name"
    log_message "  Scenario: $scenario_params"
    log_message "  Config: $config_name ($config_params)"
    log_message "  Started: $(date)"
    
    # Parse scenario parameters
    read spawn_x spawn_y spawn_yaw target_pos_x target_pos_y target_yaw <<< "$scenario_params"
    
    # Parse config parameters
    read find_anchor is_parallel run_traj_opt <<< "$config_params"
    
    # Execute the simulation with timeout to prevent hanging
    timeout 180s "$RUN_SIMULATION_SCRIPT" \
        "$WORLD" \
        "$spawn_x" "$spawn_y" "$spawn_yaw" \
        "$target_pos_x" "$target_pos_y" "$target_yaw" \
        "$IS_GOAL_COMPLETE" \
        "$find_anchor" "$is_parallel" "$run_traj_opt" \
        > /dev/null 2>&1
    
    local exit_code=$?
    
    if [ $exit_code -eq 0 ]; then
        log_message "  Status: SUCCESS"
    elif [ $exit_code -eq 124 ]; then
        log_message "  Status: TIMEOUT (180s)"
    else
        log_message "  Status: FAILED (exit code: $exit_code)"
    fi
    
    log_message "  Ended: $(date)"
    log_message ""
    
    # Mark test as completed
    mark_test_completed "$scenario_num" "$config_num"
    
    # Wait longer between tests to ensure clean shutdown and log file sync
    echo "Waiting for system cleanup and log file synchronization..."
    sleep 10
    
    return 0
}

# Main test execution
total_tests=$((NUM_SCENARIOS * NUM_CONFIGS))
test_counter=1
completed_count=0

# Count already completed tests if resuming
if [ "$RESUME_SESSION" = true ]; then
    completed_count=$(grep -c "^COMPLETED:" "$PROGRESS_FILE")
    test_counter=$((completed_count + 1))
    echo "Resuming from test $test_counter of $total_tests"
    echo ""
fi

for scenario_num in $(seq 1 $NUM_SCENARIOS); do
    scenario_params="${TEST_SCENARIOS[$((scenario_num-1))]}"
    
    for config_num in $(seq 1 $NUM_CONFIGS); do
        # Check if we should stop
        if [ "$STOP_REQUESTED" = true ]; then
            log_message "Testing interrupted by user at test $test_counter of $total_tests"
            break 2
        fi
        
        config_params="${CONFIGS[$((config_num-1))]}"
        config_name="${CONFIG_NAMES[$((config_num-1))]}"
        
        # Check if already completed
        if is_test_completed "$scenario_num" "$config_num"; then
            continue
        fi
        
        log_message "Progress: Test $test_counter of $total_tests"
        
        run_single_test "$scenario_num" "$config_num" "$scenario_params" "$config_params" "$config_name"
        
        if [ $? -ne 0 ]; then
            break 2
        fi
        
        test_counter=$((test_counter + 1))
    done
done

# Calculate final statistics
completed_count=$(grep -c "^COMPLETED:" "$PROGRESS_FILE")

log_message "========================================="
if [ "$STOP_REQUESTED" = true ] || [ $completed_count -lt $total_tests ]; then
    log_message "Batch testing interrupted: $(date)"
    log_message "Completed: $completed_count of $total_tests tests"
    log_message "Remaining: $((total_tests - completed_count)) tests"
    echo ""
    echo "========================================="
    echo "Testing interrupted!"
    echo "Completed: $completed_count of $total_tests tests"
    echo "Progress saved to: $PROGRESS_FILE"
    echo "Summary log: $LOG_FILE"
    echo ""
    echo "To resume, run this script again."
    echo "========================================="
else
    log_message "Batch testing completed: $(date)"
    log_message "All $total_tests tests finished successfully"
    log_message "Summary log saved as: $LOG_FILE"
    
    # Clean up progress file on successful completion
    if [ -f "$INTERRUPTED_FLAG" ]; then
        rm "$INTERRUPTED_FLAG"
    fi
    
    echo ""
    echo "========================================="
    echo "Batch testing completed!"
    echo "All $total_tests tests finished"
    echo "Summary log: $LOG_FILE"
    echo "Progress file: $PROGRESS_FILE"
    echo "========================================="
fi
