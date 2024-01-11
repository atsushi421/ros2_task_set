#!/bin/bash

ulimit -n 16384
source ~/ros2_caret_ws/setenv_caret.bash
source ~/ros2_thread_configurator/install/local_setup.bash
source ~/component_container_callback_isolated/install/local_setup.bash
source ~/ros2_task_set/install/local_setup.bash
ros2 launch ros2_task_set dummy_node.launch.xml
