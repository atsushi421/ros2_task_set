#!/bin/bash
set -eo pipefail

RUNTIME_BUFFER_NS="${1:-0}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DAG_SET_DIR="${SCRIPT_DIR}/ActualDAGSet"
LAUNCH_FILE="ros2_task_set dummy_node.launch.xml"
DURATION=60
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
echo "Runtime buffer: ${RUNTIME_BUFFER_NS} ns" | tee -a "${RESULT_FILE}"
echo "" | tee -a "${RESULT_FILE}"

for tu_dir in "${DAG_SET_DIR}"/TU_*; do
	tu_name="$(basename "${tu_dir}")"
	echo "========================================" | tee -a "${RESULT_FILE}"
	echo " ${tu_name}" | tee -a "${RESULT_FILE}"
	echo "========================================" | tee -a "${RESULT_FILE}"

	for yaml_file in "${tu_dir}"/case_*.yaml; do
		case_name="$(basename "${yaml_file}" .yaml)"
		# Skip taskset config files
		[[ "${case_name}" == *_taskset ]] && continue
		echo "--- ${tu_name}/${case_name} ---" | tee -a "${RESULT_FILE}"

		# Copy YAML to working directory as dags.yaml
		cp "${yaml_file}" "${SCRIPT_DIR}/dags.yaml"

		# Clean up old exec_time_logs
		rm -rf "${SCRIPT_DIR}/exec_time_logs"

		# Start thread configurator with corresponding CIE config
		cie_yaml="${tu_dir}/cie_${case_name}.yaml"
		configurator_pid=""
		configurator_output="$(mktemp)"
		cie_yaml_modified=""
		if [[ -f "${cie_yaml}" ]]; then
			# Add runtime buffer to each runtime value in the CIE config
			cie_yaml_modified="$(mktemp --suffix=.yaml)"
			awk -v buf="${RUNTIME_BUFFER_NS}" '/^[[:space:]]*runtime: [0-9]/ { match($0, /^([[:space:]]*runtime: )/, m); print m[1] ($2 + buf); next } { print }' "${cie_yaml}" >"${cie_yaml_modified}"
			setsid ros2 run agnocast_cie_thread_configurator thread_configurator_node --config-file "${cie_yaml_modified}" >"${configurator_output}" 2>&1 &
			configurator_pid=$!
		else
			echo "  [WARN] CIE config not found: ${cie_yaml}" | tee -a "${RESULT_FILE}"
		fi

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

		# Stop thread configurator (kill entire process group)
		if [[ -n "${configurator_pid}" ]]; then
			kill -INT -- "-${configurator_pid}" 2>/dev/null || true
			timeout 10 tail --pid="${configurator_pid}" -f /dev/null 2>/dev/null || {
				kill -9 -- "-${configurator_pid}" 2>/dev/null || true
			}
			wait "${configurator_pid}" 2>/dev/null || true
		fi

		# Check for SCHED_DEADLINE failure
		sched_failed=false
		if grep -q "Failed to apply SCHED_DEADLINE\|Failed to configure policy" "${configurator_output}"; then
			sched_failed=true
			echo "  失敗:" | tee -a "${RESULT_FILE}"
			grep "Failed to apply SCHED_DEADLINE\|Failed to configure policy" "${configurator_output}" |
				sed 's/.*\(Failed to apply SCHED_DEADLINE.*\)/    \1/; s/.*\(Failed to configure policy.*\)/    \1/' |
				tee -a "${RESULT_FILE}"
		fi

		# Copy response time CSVs to result directory (skip if SCHED_DEADLINE failed)
		if [[ "${sched_failed}" == false ]]; then
			case_result_dir="${RESULT_DIR}/${TIMESTAMP}/${tu_name}/${case_name}"
			rt_csvs=("${SCRIPT_DIR}"/exec_time_logs/response_time_task*.csv)
			if [[ -e "${rt_csvs[0]}" ]]; then
				mkdir -p "${case_result_dir}"
				cp "${SCRIPT_DIR}"/exec_time_logs/response_time_task*.csv "${case_result_dir}/"
				echo "  Response time CSVs saved to ${case_result_dir}/" | tee -a "${RESULT_FILE}"
				for rt_csv in "${case_result_dir}"/response_time_task*.csv; do
					echo "    $(basename "${rt_csv}"): $(($(wc -l <"${rt_csv}") - 1)) samples" | tee -a "${RESULT_FILE}"
				done
			else
				echo "  [WARN] No response time CSVs found" | tee -a "${RESULT_FILE}"
			fi
		fi

		rm -f "${output_file}" "${configurator_output}" "${cie_yaml_modified}"
		echo "" | tee -a "${RESULT_FILE}"
	done
done

echo "=== Benchmark finished at $(date) ===" | tee -a "${RESULT_FILE}"
echo "Results saved to: ${RESULT_FILE}"
