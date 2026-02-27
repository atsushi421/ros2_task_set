#!/bin/bash
set -euo pipefail

# Disable all CPU cores except the specified ones.
# Usage: sudo ./disable_cpus.sh 0 1 2
# Requires root privileges.

if [[ $EUID -ne 0 ]]; then
	echo "Error: must be run as root" >&2
	exit 1
fi

if [[ $# -eq 0 ]]; then
	echo "Usage: $0 <core_id> [core_id ...]" >&2
	echo "Example: $0 0 1 2" >&2
	exit 1
fi

KEEP_CORES=("$@")

for cpu_dir in /sys/devices/system/cpu/cpu[0-9]*; do
	cpu_id="${cpu_dir##*cpu}"
	# Skip non-numeric entries
	[[ "${cpu_id}" =~ ^[0-9]+$ ]] || continue

	# CPU 0 has no online file
	[[ ! -f "${cpu_dir}/online" ]] && continue

	# Check if this core should stay enabled
	keep=false
	for k in "${KEEP_CORES[@]}"; do
		if [[ "${cpu_id}" -eq "${k}" ]]; then
			keep=true
			break
		fi
	done

	if [[ "${keep}" == false ]]; then
		echo 0 >"${cpu_dir}/online"
		echo "Disabled CPU ${cpu_id}"
	else
		echo 1 >"${cpu_dir}/online"
		echo "Enabled CPU ${cpu_id}"
	fi
done

echo ""
echo "Online CPUs: $(cat /sys/devices/system/cpu/online)"
