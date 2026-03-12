#!/bin/bash

set -e

source /opt/ros/one/setup.bash
source /home/chen/projects/hydrus_planning_ws/devel/setup.bash
source /home/chen/projects/motion_planning_ws/devel/setup.bash


roscd squeeze_navigation
cd bash
./run_batch_simulations.sh

sleep 60

roscd motion_planner
cd bash
./run_batch_tests.sh

sleep 30

