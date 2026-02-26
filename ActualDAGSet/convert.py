#!/usr/bin/env python3
"""Convert case_*.yaml files from ROS2 callback format to DAG-scheduling taskset format."""

import glob
import os
import yaml


def convert(input_path, output_path):
    with open(input_path) as f:
        data = yaml.safe_load(f)

    callbacks = data["callbacks"]
    communications = data["communications"]

    # Group callbacks into DAGs. A new DAG starts at each callback with period_ms.
    dags = []
    current_dag = None
    for cb in callbacks:
        if "period_ms" in cb:
            current_dag = {"period": cb["period_ms"], "deadline": None, "nodes": [], "node_ids": set()}
            dags.append(current_dag)
        node = {"id": cb["id"], "c": cb["execution_time_ms"]}
        if "deadline_ms" in cb:
            current_dag["deadline"] = cb["deadline_ms"]
        current_dag["nodes"].append(node)
        current_dag["node_ids"].add(cb["id"])

    # Build tasks in target format
    tasks = []
    for dag in dags:
        # Map global IDs to local 0-indexed IDs
        global_ids = [n["id"] for n in dag["nodes"]]
        g2l = {gid: local for local, gid in enumerate(global_ids)}

        # Use deadline if present, otherwise use period as implicit deadline
        deadline = dag["deadline"] if dag["deadline"] is not None else dag["period"]

        task = {
            "t": dag["period"],
            "d": deadline,
            "vertices": [{"id": g2l[n["id"]], "c": n["c"]} for n in dag["nodes"]],
            "edges": [],
        }

        # Filter communications for this DAG
        for comm in communications:
            if comm["from"] in dag["node_ids"] and comm["to"] in dag["node_ids"]:
                task["edges"].append({"from": g2l[comm["from"]], "to": g2l[comm["to"]]})

        tasks.append(task)

    output = {"tasks": tasks}
    with open(output_path, "w") as f:
        yaml.dump(output, f, default_flow_style=False, sort_keys=False)


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    input_files = sorted(glob.glob(os.path.join(base_dir, "**", "case_*.yaml"), recursive=True))

    for input_path in input_files:
        dirname = os.path.dirname(input_path)
        basename = os.path.basename(input_path)
        name = os.path.splitext(basename)[0]
        output_path = os.path.join(dirname, f"{name}_taskset.yaml")
        convert(input_path, output_path)
        print(f"Converted: {input_path} -> {output_path}")


if __name__ == "__main__":
    main()
