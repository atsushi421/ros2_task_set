#!/bin/bash
set -euo pipefail

# Kill all remaining processes spawned by run_benchmark.sh.

kill_procs() {
	local pattern="$1"
	local pids
	pids="$(pgrep -f "${pattern}" 2>/dev/null || true)"
	if [[ -n "${pids}" ]]; then
		echo "Killing: ${pattern}"
		echo "${pids}" | while read -r pid; do
			echo "  PID ${pid}: $(ps -p "${pid}" -o args= 2>/dev/null || echo '(already exited)')"
			kill -INT "${pid}" 2>/dev/null || true
		done
	fi
}

force_kill_procs() {
	local pattern="$1"
	local pids
	pids="$(pgrep -f "${pattern}" 2>/dev/null || true)"
	if [[ -n "${pids}" ]]; then
		echo "Force killing remaining: ${pattern}"
		echo "${pids}" | while read -r pid; do
			kill -9 "${pid}" 2>/dev/null || true
		done
	fi
}

# Target processes
PATTERNS=(
	"run_benchmark.sh"
	"dummy_node.launch.xml"
	"thread_configurator_node"
	"agnocast_component_container"
	"sample_node"
)

# First pass: SIGINT for graceful shutdown
for p in "${PATTERNS[@]}"; do
	kill_procs "${p}"
done

echo ""
echo "Waiting 3 seconds for graceful shutdown..."
sleep 3

# Second pass: SIGKILL for anything still alive
for p in "${PATTERNS[@]}"; do
	force_kill_procs "${p}"
done

echo ""
echo "Done."
