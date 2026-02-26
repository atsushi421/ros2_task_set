#!/bin/bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DAG_SET_DIR="${SCRIPT_DIR}/ActualDAGSet"
LAUNCH_FILE="ros2_task_set dummy_node.launch.xml"
DURATION=5
RESULT_DIR="${SCRIPT_DIR}/benchmark_results"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
RESULT_FILE="${RESULT_DIR}/results_${TIMESTAMP}.txt"

# Source ROS 2 workspace (nounset disabled for compatibility with setup.bash)
set +u
source "${SCRIPT_DIR}/install/setup.bash"
set -u

mkdir -p "${RESULT_DIR}"

echo "=== Benchmark started at $(date) ===" | tee "${RESULT_FILE}"
echo "Duration per case: ${DURATION}s" | tee -a "${RESULT_FILE}"
echo "" | tee -a "${RESULT_FILE}"

for tu_dir in "${DAG_SET_DIR}"/TU_*; do
	tu_name="$(basename "${tu_dir}")"
	echo "========================================" | tee -a "${RESULT_FILE}"
	echo " ${tu_name}" | tee -a "${RESULT_FILE}"
	echo "========================================" | tee -a "${RESULT_FILE}"

	for yaml_file in "${tu_dir}"/*.yaml; do
		case_name="$(basename "${yaml_file}" .yaml)"
		echo "--- ${tu_name}/${case_name} ---" | tee -a "${RESULT_FILE}"

		# Copy YAML to working directory as dags.yaml
		cp "${yaml_file}" "${SCRIPT_DIR}/dags.yaml"

		# Clean up old exec_time_logs
		rm -rf "${SCRIPT_DIR}/exec_time_logs"

		# Run the launch file in a new process group, capturing all output
		output_file="$(mktemp)"
		setsid ros2 launch ${LAUNCH_FILE} >"${output_file}" 2>&1 &
		launch_pid=$!

		# Wait for the specified duration
		sleep "${DURATION}"

		# Send SIGINT to the entire process group for clean shutdown
		kill -INT -- "-${launch_pid}" 2>/dev/null || true

		# Wait for the process to exit (with timeout)
		timeout 30 tail --pid="${launch_pid}" -f /dev/null 2>/dev/null || {
			echo "  [WARN] Process did not exit cleanly, sending SIGKILL" | tee -a "${RESULT_FILE}"
			kill -9 -- "-${launch_pid}" 2>/dev/null || true
		}
		wait "${launch_pid}" 2>/dev/null || true

		# Extract and record Response Time Statistics
		if grep -q "Response Time Statistics" "${output_file}"; then
			sed -n '/=== Response Time Statistics ===/,$ p' "${output_file}" | tee -a "${RESULT_FILE}"
		else
			echo "  [WARN] No Response Time Statistics found" | tee -a "${RESULT_FILE}"
		fi

		rm -f "${output_file}"
		echo "" | tee -a "${RESULT_FILE}"
	done
done

echo "=== Benchmark finished at $(date) ===" | tee -a "${RESULT_FILE}"
echo "Results saved to: ${RESULT_FILE}"
