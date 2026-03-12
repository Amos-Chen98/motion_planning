#!/usr/bin/env python3
"""
Test Scenario Generator for Motion Planning

This script generates random test scenarios with initial and target positions/orientations
within specified boundaries. Each scenario consists of:
- Initial position (x, y) and orientation (yaw)
- Target position (x, y) and orientation (yaw)

The output format is compatible with bash scripts and includes quotation marks.
"""

import random
import math

# ============================================================================
# KEY CONFIGURATION PARAMETERS
# ============================================================================
NUM_SCENARIOS = 20

# Initial position boundaries
INITIAL_X_MIN = 2.0
INITIAL_X_MAX = 2.8
INITIAL_Y_MIN = 0.0
INITIAL_Y_MAX = 0.0
INITIAL_YAW_MIN = 2.6
INITIAL_YAW_MAX = 2.6

# Target position boundaries
TARGET_X_MIN = -0.8
TARGET_X_MAX = -0.8
TARGET_Y_MIN = 1.2
TARGET_Y_MAX = 1.2
TARGET_YAW_MIN = 3.141
TARGET_YAW_MAX = 3.141

# Random seed (set to None for random results, or a number for reproducible results)
RANDOM_SEED = None

# ============================================================================

# Set random seed if specified
if RANDOM_SEED is not None:
    random.seed(RANDOM_SEED)

# Generate scenarios data
scenarios = []
transformed_scenarios = []

for i in range(NUM_SCENARIOS):
    # Generate random initial position and orientation
    initial_x = random.uniform(INITIAL_X_MIN, INITIAL_X_MAX)
    initial_y = random.uniform(INITIAL_Y_MIN, INITIAL_Y_MAX)
    initial_yaw = random.uniform(INITIAL_YAW_MIN, INITIAL_YAW_MAX)
    
    # Generate random target position and orientation
    target_x = random.uniform(TARGET_X_MIN, TARGET_X_MAX)
    target_y = random.uniform(TARGET_Y_MIN, TARGET_Y_MAX)
    target_yaw = random.uniform(TARGET_YAW_MIN, TARGET_YAW_MAX)
    
    # Store original scenario
    scenarios.append((initial_x, initial_y, initial_yaw, target_x, target_y, target_yaw))
    
    # Create transformed scenario (rotated 180 degrees around origin)
    transformed_x = -initial_x
    transformed_y = -initial_y
    transformed_yaw = (initial_yaw + math.pi) % (2 * math.pi)  # Add pi and normalize
    
    transformed_scenarios.append((transformed_x, transformed_y, transformed_yaw))

# Print first matrix (original scenarios)
print("# Generated Test Scenarios")
print("# Format: initial_x initial_y initial_yaw target_x target_y target_yaw")
print()

for scenario in scenarios:
    initial_x, initial_y, initial_yaw, target_x, target_y, target_yaw = scenario
    print(f'    "{initial_x:.2f} {initial_y:.2f} {initial_yaw:.3f} {target_x:.2f} {target_y:.2f} {target_yaw:.3f}"')

print()
print("# Transformed Initial Positions (rotated 180° around origin)")
print("# Format: initial_x initial_y initial_yaw")
print()

# Print second matrix (transformed initial positions)
for transformed in transformed_scenarios:
    transformed_x, transformed_y, transformed_yaw = transformed
    print(f'    "{transformed_x:.2f} {transformed_y:.2f} {transformed_yaw:.3f}"')
