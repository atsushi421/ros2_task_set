#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash
ulimit -n 16384
ros2 caret record -f 10000 --light -p ~/ros2_task_set/caret_trace_data
