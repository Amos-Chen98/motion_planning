#!/bin/bash

# Batch test script for hydrus planning simulation
# This script runs test scenarios with multiple configurations each
# Total simulation runs = number of predefined scenarios * NUM_CONFIGS

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
RUN_SIMULATION_SCRIPT="$SCRIPT_DIR/run_simulation_hyperparams.sh"

# Constant parameters
WORLD="gap"
IS_GOAL_COMPLETE="false"

# Script configuration variables
NUM_CONFIGS=25  # 5 sample_density values × 5 transform_attempt_num values

# Hyperparameter values to test
declare -a SAMPLE_DENSITY_VALUES=(5 10 20 50 100)
declare -a TRANSFORM_ATTEMPT_NUM_VALUES=(5 10 15 20 30)

# Predefined test scenarios (manually specified)
# Format: "spawn_x spawn_y spawn_yaw target_pos_x target_pos_y target_yaw"
declare -a PREDEFINED_TEST_SCENARIOS=(
    "2.08 0.00 2.600 -0.80 1.20 3.141"
    "2.41 0.00 2.600 -0.80 1.20 3.141"
    "2.14 0.00 2.600 -0.80 1.20 3.141"
    "2.47 0.00 2.600 -0.80 1.20 3.141"
    "2.79 0.00 2.600 -0.80 1.20 3.141"
    "2.46 0.00 2.600 -0.80 1.20 3.141"
    "2.36 0.00 2.600 -0.80 1.20 3.141"
    "2.79 0.00 2.600 -0.80 1.20 3.141"
    "2.52 0.00 2.600 -0.80 1.20 3.141"
    "2.06 0.00 2.600 -0.80 1.20 3.141"
    "2.51 0.00 2.600 -0.80 1.20 3.141"
    "2.24 0.00 2.600 -0.80 1.20 3.141"
    "2.43 0.00 2.600 -0.80 1.20 3.141"
    "2.09 0.00 2.600 -0.80 1.20 3.141"
    "2.12 0.00 2.600 -0.80 1.20 3.141"
    "2.67 0.00 2.600 -0.80 1.20 3.141"
    "2.42 0.00 2.600 -0.80 1.20 3.141"
    "2.04 0.00 2.600 -0.80 1.20 3.141"
    "2.11 0.00 2.600 -0.80 1.20 3.141"
    "2.44 0.00 2.600 -0.80 1.20 3.141"
)

# Test configurations - generate all combinations of sample_density and transform_attempt_num
declare -a CONFIGS=()
declare -a CONFIG_NAMES=()

# Generate all 25 combinations
config_index=0
for sample_density in "${SAMPLE_DENSITY_VALUES[@]}"; do
    for transform_attempt_num in "${TRANSFORM_ATTEMPT_NUM_VALUES[@]}"; do
        # Fixed parameters for all configurations
        CONFIGS+=("true true true $sample_density $transform_attempt_num")
        CONFIG_NAMES+=("SD${sample_density}_TA${transform_attempt_num}")
        config_index=$((config_index + 1))
    done
done

# Check if run_simulation_hyperparams.sh exists
if [ ! -f "$RUN_SIMULATION_SCRIPT" ]; then
    echo "Error: run_simulation_hyperparams.sh not found at $RUN_SIMULATION_SCRIPT"
    exit 1
fi

# Make sure run_simulation_hyperparams.sh is executable
chmod +x "$RUN_SIMULATION_SCRIPT"

# Load predefined test scenarios
NUM_SCENARIOS=${#PREDEFINED_TEST_SCENARIOS[@]}
echo "Loading $NUM_SCENARIOS predefined test scenarios..."
declare -a TEST_SCENARIOS

for i in $(seq 0 $((NUM_SCENARIOS-1))); do
    TEST_SCENARIOS+=("${PREDEFINED_TEST_SCENARIOS[$i]}")
    
    # Parse and display scenario parameters for verification
    read spawn_x spawn_y spawn_yaw target_pos_x target_pos_y target_yaw <<< "${PREDEFINED_TEST_SCENARIOS[$i]}"
    echo "Scenario $((i+1)): spawn=($spawn_x, $spawn_y, $spawn_yaw) target=($target_pos_x, $target_pos_y, $target_yaw)"
done

echo ""
echo "Starting batch testing..."
echo "Total tests to run: $(($NUM_SCENARIOS * $NUM_CONFIGS)) = $((NUM_SCENARIOS * NUM_CONFIGS))"
echo ""

# Create bash_logs directory if it doesn't exist
BASH_LOGS_DIR="$SCRIPT_DIR/bash_logs"
mkdir -p "$BASH_LOGS_DIR"

# Create summary log file
LOG_FILE="$BASH_LOGS_DIR/batch_test_summary_$(date +%Y%m%d_%H%M%S).txt"

# Function to log messages
log_message() {
    local message="$1"
    echo "$message" | tee -a "$LOG_FILE"
}

# Create summary log with all test info at the beginning
{
    echo "Batch Test Summary"
    echo "=================="
    echo "Date: $(date)"
    echo "Total Scenarios: $NUM_SCENARIOS"
    echo "Configurations per Scenario: $NUM_CONFIGS"
    echo "Total Tests: $(($NUM_SCENARIOS * $NUM_CONFIGS))"
    echo ""
    echo "Test Scenarios ($NUM_SCENARIOS):"
    for i in $(seq 1 $NUM_SCENARIOS); do
        echo "  Scenario $i: ${TEST_SCENARIOS[$((i-1))]}"
    done
    echo ""
    echo "Test Configurations ($NUM_CONFIGS):"
    for i in $(seq 1 $NUM_CONFIGS); do
        echo "  Config $i: ${CONFIG_NAMES[$((i-1))]} - ${CONFIGS[$((i-1))]}"
    done
    echo ""
    echo "Test Execution Log:"
    echo "==================="
    echo ""
} > "$LOG_FILE"

log_message "Batch Test Started: $(date)"
log_message ""

# Function to run a single test
run_single_test() {
    local scenario_num=$1
    local config_num=$2
    local scenario_params=$3
    local config_params=$4
    local config_name=$5
    
    local test_name="scenario_${scenario_num}_config_${config_num}"
    
    log_message "Running Test: $test_name"
    log_message "  Scenario: $scenario_params"
    log_message "  Config: $config_name ($config_params)"
    log_message "  Started: $(date)"
    
    # Parse scenario parameters
    read spawn_x spawn_y spawn_yaw target_pos_x target_pos_y target_yaw <<< "$scenario_params"
    
    # Parse config parameters
    read find_anchor is_parallel run_traj_opt sample_density transform_attempt_num <<< "$config_params"
    
    # Execute the simulation with timeout to prevent hanging
    timeout 180s "$RUN_SIMULATION_SCRIPT" \
        "$WORLD" \
        "$spawn_x" "$spawn_y" "$spawn_yaw" \
        "$target_pos_x" "$target_pos_y" "$target_yaw" \
        "$IS_GOAL_COMPLETE" \
        "$find_anchor" "$is_parallel" "$run_traj_opt" \
        "$sample_density" "$transform_attempt_num" \
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
    
    # Wait a bit between tests to ensure clean shutdown
    sleep 5
}

# Main test loop
test_counter=1
total_tests=$(($NUM_SCENARIOS * $NUM_CONFIGS))

for scenario_num in $(seq 1 $NUM_SCENARIOS); do
    scenario_params="${TEST_SCENARIOS[$((scenario_num-1))]}"
    
    for config_num in $(seq 1 $NUM_CONFIGS); do
        config_params="${CONFIGS[$((config_num-1))]}"
        config_name="${CONFIG_NAMES[$((config_num-1))]}"
        
        log_message "Progress: Test $test_counter of $total_tests"
        
        run_single_test "$scenario_num" "$config_num" "$scenario_params" "$config_params" "$config_name"
        
        test_counter=$((test_counter + 1))
    done
done

log_message "Batch testing completed: $(date)"
log_message "Summary log saved as: $LOG_FILE"

echo ""
echo "========================================="
echo "Batch testing completed!"
echo "Summary log: $LOG_FILE"
echo "========================================="
