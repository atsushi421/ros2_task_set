# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Task Set is a performance benchmarking tool that simulates callback DAGs (Directed Acyclic Graphs) for measuring real-time scheduling behavior. It runs as a composable node inside Agnocast's callback-isolated executor (`agnocast_component_container_cie`).

## Build

```bash
git submodule update --init --recursive
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --base-paths . agnocast/src --packages-up-to ros2_task_set agnocast_components
```

## Run

Place `dags.yaml` in the current working directory, then:

```bash
source install/setup.bash
ros2 launch ros2_task_set dummy_node.launch.xml
```

## Architecture

- **Single composable node** (`DummyNode` in `src/dummy_node.cpp`): Parses a YAML DAG definition and creates timer/subscription callbacks that simulate workloads with configurable execution times.
- **Launch file** (`launch/dummy_node.launch.xml`): Loads `DummyNode` into `agnocast_component_container_cie` as a composable node plugin.
- **Agnocast submodule** (`agnocast/`): Provides the callback-isolated executor and zero-copy IPC middleware. Built alongside the main package via `--base-paths . agnocast/src`.

### DAG YAML format

Callbacks with `period_ms` become timers (DAG roots). Callbacks without it become subscriptions triggered via `communications` edges. Optional `deadline_ms` enables latency tracking and deadline-miss statistics at shutdown.

### Performance measurement

- Execution times are logged to CSV files in `exec_time_logs/`.
- `evaluate.ipynb` visualizes callback scheduling from those CSVs.
- Thread scheduling policy (SCHED_OTHER/FIFO/RR/DEADLINE) is recorded per callback invocation.

## Code Style

- `.clang-format`: Google-based, 100-char column limit.
- `.clang-tidy`: Extensive checks enabled — run before submitting changes.
