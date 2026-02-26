#!/bin/bash
#
# Generate CIE thread configurator templates for each DAG case.
#
# For each case_N.yaml in ActualDAGSet/, this script:
#   1. Starts the prerun node (subscribes to callback_group_info topic)
#   2. Launches the application (publishes callback group info at startup)
#   3. Waits for discovery, then cleanly shuts down both
#   4. Saves the generated template.yaml as cie_case_N.yaml
#
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"
DAG_SET_DIR="${PROJECT_DIR}/ActualDAGSet"
LAUNCH_FILE="ros2_task_set dummy_node.launch.xml"
WAIT_BEFORE_LAUNCH=2
WAIT_FOR_DISCOVERY=5

# PIDs to track for cleanup
PRERUN_PID=""
LAUNCH_PGID=""

cleanup() {
    echo "[cleanup] Stopping all child processes..."

    # Stop prerun node (process group)
    if [[ -n "${PRERUN_PID}" ]] && kill -0 "-${PRERUN_PID}" 2>/dev/null; then
        kill -INT "-${PRERUN_PID}" 2>/dev/null || true
        sleep 1
        kill -9 "-${PRERUN_PID}" 2>/dev/null || true
    fi

    # Stop launch process group
    if [[ -n "${LAUNCH_PGID}" ]] && kill -0 "-${LAUNCH_PGID}" 2>/dev/null; then
        kill -INT "-${LAUNCH_PGID}" 2>/dev/null || true
        sleep 2
        kill -9 "-${LAUNCH_PGID}" 2>/dev/null || true
    fi

    # Final sweep: kill any lingering thread_configurator or container processes
    pkill -f "thread_configurator_node --prerun" 2>/dev/null || true
    pkill -f "agnocast_component_container_cie" 2>/dev/null || true
    sleep 1
    pkill -9 -f "thread_configurator_node --prerun" 2>/dev/null || true
    pkill -9 -f "agnocast_component_container_cie" 2>/dev/null || true

    PRERUN_PID=""
    LAUNCH_PGID=""
}

trap cleanup EXIT

# Source ROS 2 workspace
set +u
source "${PROJECT_DIR}/install/setup.bash"
set -u

cd "${PROJECT_DIR}"

# Collect case files sorted by case number
mapfile -t case_files < <(
    find "${DAG_SET_DIR}" -name 'case_*.yaml' ! -name 'cie_*' | sort -V
)

echo "=== Generating CIE configs for ${#case_files[@]} cases ==="
echo ""

success=0
fail=0

for case_file in "${case_files[@]}"; do
    case_name="$(basename "${case_file}" .yaml)"
    case_num="${case_name#case_}"
    tu_dir="$(dirname "${case_file}")"
    tu_name="$(basename "${tu_dir}")"
    output_file="${tu_dir}/cie_case_${case_num}.yaml"

    echo "--- ${tu_name}/${case_name} ---"

    # Prepare dags.yaml
    cp "${case_file}" "${PROJECT_DIR}/dags.yaml"
    rm -f "${PROJECT_DIR}/template.yaml"

    # 1. Start prerun node FIRST (must be ready before launch publishes info)
    #    Use setsid so SIGINT reaches the actual C++ process (ros2 run uses subprocess.Popen,
    #    so kill -INT on the wrapper PID alone won't reach the child node).
    setsid ros2 run agnocast_cie_thread_configurator thread_configurator_node --prerun > /dev/null 2>&1 &
    PRERUN_PID=$!

    sleep "${WAIT_BEFORE_LAUNCH}"

    # 2. Launch the application in a new process group
    setsid ros2 launch ${LAUNCH_FILE} > /dev/null 2>&1 &
    LAUNCH_PGID=$!

    # 3. Wait for callback group discovery
    sleep "${WAIT_FOR_DISCOVERY}"

    # 4. Stop prerun process group with SIGINT (triggers template.yaml write)
    kill -INT "-${PRERUN_PID}" 2>/dev/null || true
    # Wait for prerun to finish writing template.yaml
    for i in $(seq 1 10); do
        if ! kill -0 "-${PRERUN_PID}" 2>/dev/null; then
            break
        fi
        sleep 0.5
    done
    # Force kill if still alive
    kill -9 "-${PRERUN_PID}" 2>/dev/null || true
    wait "${PRERUN_PID}" 2>/dev/null || true
    PRERUN_PID=""

    # 5. Stop launch process group
    kill -INT "-${LAUNCH_PGID}" 2>/dev/null || true
    for i in $(seq 1 10); do
        if ! kill -0 "-${LAUNCH_PGID}" 2>/dev/null; then
            break
        fi
        sleep 0.5
    done
    kill -9 "-${LAUNCH_PGID}" 2>/dev/null || true
    wait "${LAUNCH_PGID}" 2>/dev/null || true
    LAUNCH_PGID=""

    # 6. Validate and save result
    if [[ -f "${PROJECT_DIR}/template.yaml" ]]; then
        cbg_count=$(grep -c -- '- id:' "${PROJECT_DIR}/template.yaml" || true)
        if [[ "${cbg_count}" -gt 0 ]]; then
            cp "${PROJECT_DIR}/template.yaml" "${output_file}"
            echo "  -> OK: ${output_file} (${cbg_count} callback groups)"
            success=$((success + 1))
        else
            echo "  -> WARN: template.yaml generated but callback_groups is empty"
            cp "${PROJECT_DIR}/template.yaml" "${output_file}"
            fail=$((fail + 1))
        fi
    else
        echo "  -> ERROR: template.yaml was not generated"
        fail=$((fail + 1))
    fi

    echo ""
done

echo "=== Done: ${success} succeeded, ${fail} failed ==="
