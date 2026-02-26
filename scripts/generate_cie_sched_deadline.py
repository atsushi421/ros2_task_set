#!/usr/bin/env python3
"""Regenerate cie_case_*.yaml files with unique callback group IDs and SCHED_DEADLINE.

Usage:
    python3 scripts/generate_cie_sched_deadline.py ActualDAGSet
"""
import yaml
import re
import os
import sys
import glob

NODE_NAME = "/dummy_node"

HEADER = """\
hardware_info:
  cpu_family: 6
  cpu_max_mhz: 2101.0000
  cpu_min_mhz: 800.0000
  frequency_boost: enabled
  model: 85
  model_name: Intel(R) Xeon(R) Silver 4216 CPU @ 2.10GHz
  threads_per_core: 1
rt_throttling:
  runtime_us: 950000
  period_us: 1000000
callback_groups:
  - id: {node}@Service({node}/describe_parameters)@Service({node}/get_parameter_types)@Service({node}/get_parameters)@Service({node}/list_parameters)@Service({node}/set_parameters)@Service({node}/set_parameters_atomically)@Subscription(/parameter_events)
    domain_id: 0
    affinity: ~
    policy: SCHED_OTHER
    priority: 0
""".format(node=NODE_NAME)

ENTRY_TEMPLATE = """\
  - id: {id}
    domain_id: 0
    affinity: ~
    policy: SCHED_DEADLINE
    runtime: {runtime}
    period: {period}
    deadline: {deadline}
"""

FOOTER = """\
non_ros_threads:
  []
"""


def get_dag_membership(callbacks, communications):
    adj = {cb['id']: set() for cb in callbacks}
    for comm in communications:
        f, t = comm['from'], comm['to']
        adj[f].add(t)
        adj[t].add(f)

    visited = set()
    dag_membership = {}
    dag_index = 0

    for cb in callbacks:
        cb_id = cb['id']
        if cb_id in visited:
            continue
        queue = [cb_id]
        visited.add(cb_id)
        component = []
        while queue:
            node = queue.pop(0)
            component.append(node)
            for neighbor in adj.get(node, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        for node in component:
            dag_membership[node] = dag_index
        dag_index += 1

    return dag_membership


def make_cbg_id(parts):
    """Build CIE callback group ID from sorted entity parts."""
    sorted_parts = sorted(parts)
    return NODE_NAME + "@" + "@".join(sorted_parts)


def process_case(case_yaml_path, cie_yaml_path):
    with open(case_yaml_path) as f:
        case_data = yaml.safe_load(f)

    callbacks = case_data['callbacks']
    communications = case_data['communications']

    dag_membership = get_dag_membership(callbacks, communications)

    # For each DAG, find source period and sink deadline
    dag_info = {}
    for cb in callbacks:
        dag_idx = dag_membership[cb['id']]
        if dag_idx not in dag_info:
            dag_info[dag_idx] = {}
        if 'period_ms' in cb:
            dag_info[dag_idx]['source_period_ms'] = cb['period_ms']
        if 'deadline_ms' in cb:
            dag_info[dag_idx]['sink_deadline_ms'] = cb['deadline_ms']

    # Build incoming edges map
    incoming_edges = {}
    for comm in communications:
        to_id = comm['to']
        from_id = comm['from']
        if to_id not in incoming_edges:
            incoming_edges[to_id] = []
        if from_id not in incoming_edges[to_id]:
            incoming_edges[to_id].append(from_id)

    # Generate CIE entries
    entries = []  # list of (id_str, runtime_ns, period_ns, deadline_ns)

    for cb in callbacks:
        cb_id = cb['id']
        dag_idx = dag_membership[cb_id]
        source_period_ns = dag_info[dag_idx]['source_period_ms'] * 1000000
        sink_deadline_ns = dag_info[dag_idx]['sink_deadline_ms'] * 1000000
        runtime_ns = cb['execution_time_ms'] * 1000000

        if 'period_ms' in cb:
            # Timer callback
            period_ns = cb['period_ms'] * 1000000
            cbg_id = make_cbg_id([
                f"Subscription(/dummy_cbg_{cb_id})",
                f"Timer({period_ns})",
            ])
            entries.append((cbg_id, runtime_ns, period_ns, sink_deadline_ns))
        else:
            sources = incoming_edges.get(cb_id, [])
            if len(sources) == 1:
                # Single-source subscription
                cbg_id = make_cbg_id([
                    f"Subscription(/dummy_cbg_{cb_id})",
                    f"Subscription(/topic_{sources[0]})",
                ])
                entries.append((cbg_id, runtime_ns, source_period_ns, sink_deadline_ns))
            elif len(sources) >= 2:
                # Multi-source (synchronizer) - each subscription gets its own entry
                for i, src in enumerate(sources):
                    if i == 0:
                        marker_id = str(cb_id)
                    else:
                        marker_id = f"{cb_id}_{i}"
                    cbg_id = make_cbg_id([
                        f"Subscription(/dummy_cbg_{marker_id})",
                        f"Subscription(/topic_{src})",
                    ])
                    entries.append((cbg_id, runtime_ns, source_period_ns, sink_deadline_ns))

    # Sort entries alphabetically by ID
    entries.sort(key=lambda e: e[0])

    # Write the CIE yaml
    with open(cie_yaml_path, 'w') as f:
        f.write(HEADER)
        f.write("\n")
        for cbg_id, runtime_ns, period_ns, deadline_ns in entries:
            f.write(ENTRY_TEMPLATE.format(
                id=cbg_id,
                runtime=runtime_ns,
                period=period_ns,
                deadline=deadline_ns,
            ))
            f.write("\n")
        f.write(FOOTER)

    print(f"Updated: {cie_yaml_path}")


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <dag_set_dir>", file=sys.stderr)
        sys.exit(1)

    base_dir = sys.argv[1]
    if not os.path.isdir(base_dir):
        print(f"Error: {base_dir} is not a directory", file=sys.stderr)
        sys.exit(1)

    cie_paths = sorted(glob.glob(os.path.join(base_dir, "**/cie_case_*.yaml"), recursive=True))
    if not cie_paths:
        print(f"No cie_case_*.yaml files found in {base_dir}", file=sys.stderr)
        sys.exit(1)

    for cie_path in cie_paths:
        dir_name = os.path.dirname(cie_path)
        case_num = re.search(r'cie_case_(\d+)\.yaml', cie_path).group(1)
        case_path = os.path.join(dir_name, f"case_{case_num}.yaml")
        if os.path.exists(case_path):
            process_case(case_path, cie_path)
        else:
            print(f"WARNING: {case_path} not found for {cie_path}")


if __name__ == "__main__":
    main()
