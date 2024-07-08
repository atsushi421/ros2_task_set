#!/bin/bash

ulimit -n 65535
ulimit -c unlimited
source /home/atsushi/ros2_caret_ws/setenv_caret.bash
source /home/atsushi/ros2_thread_configurator/install/local_setup.bash
source /home/atsushi/component_container_callback_isolated/install/local_setup.bash
source /home/atsushi/ros2_task_set/install/local_setup.bash
ros2 launch ros2_task_set dummy_node.launch.xml
