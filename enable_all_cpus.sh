#!/bin/bash
set -euo pipefail

# Re-enable all CPU cores.
# Requires root privileges.

if [[ $EUID -ne 0 ]]; then
	echo "Error: must be run as root" >&2
	exit 1
fi

for cpu_dir in /sys/devices/system/cpu/cpu[0-9]*; do
	cpu_id="${cpu_dir##*cpu}"
	[[ "${cpu_id}" =~ ^[0-9]+$ ]] || continue
	[[ ! -f "${cpu_dir}/online" ]] && continue

	if [[ "$(cat "${cpu_dir}/online")" -eq 0 ]]; then
		echo 1 >"${cpu_dir}/online"
		echo "Enabled CPU ${cpu_id}"
	fi
done

echo ""
echo "Online CPUs: $(cat /sys/devices/system/cpu/online)"
