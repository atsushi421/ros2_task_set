#!/bin/bash

BUILD_TYPE=${1:-Release}

rm -r build/ install/ log/
source ~/ros2_thread_configurator/install/local_setup.bash
source ~/component_container_callback_isolated/install/local_setup.bash
source ~/ros2_caret_ws/install/local_setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE
