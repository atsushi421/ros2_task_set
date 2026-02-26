#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TARGET="${PROJECT_DIR}/build/agnocast_cie_thread_configurator/thread_configurator_node"

if [ ! -f "${TARGET}" ]; then
	echo "Error: ${TARGET} not found. Build the workspace first." >&2
	exit 1
fi

# Set cap_sys_nice capability so thread_configurator_node can call sched_setscheduler(2)
sudo setcap cap_sys_nice+ep "${TARGET}"
echo "Set cap_sys_nice+ep on ${TARGET}"

# After setcap, the dynamic linker ignores LD_LIBRARY_PATH for security.
# Register library paths via ldconfig so shared libraries can still be resolved.
ROS_DISTRO="${ROS_DISTRO:-humble}"
CONF_FILE="/etc/ld.so.conf.d/ros2_task_set.conf"

ARCH="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || echo "x86_64-linux-gnu")"

cat <<EOF | sudo tee "${CONF_FILE}" >/dev/null
/opt/ros/${ROS_DISTRO}/lib
/opt/ros/${ROS_DISTRO}/lib/${ARCH}
${PROJECT_DIR}/install/agnocast_cie_config_msgs/lib
EOF

echo "Wrote ${CONF_FILE}"

sudo ldconfig
echo "Done. Capability and ldconfig configured successfully."
