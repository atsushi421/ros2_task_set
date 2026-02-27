# ROS 2 Task Set

A performance benchmarking tool that simulates callback DAGs (Directed Acyclic Graphs) for measuring real-time scheduling behavior.

## Setup

### 1. Clone the repository

```bash
git clone https://github.com/autowarefoundation/ros2_task_set.git
cd ros2_task_set
git submodule update --init --recursive
```

### 2. Install dependencies

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
bash agnocast/scripts/setup
```

### 4. Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --base-paths . agnocast/src --packages-up-to ros2_task_set agnocast_components
```

## Run

Place `dags.yaml` (see [Usage](#usage)) in the current directory, then:

```bash
source install/setup.bash
ros2 launch ros2_task_set dummy_node.launch.xml
```

## Run Benchmark

`run_benchmark.sh` iterates over all DAG cases in `ActualDAGSet/` and records response-time CSVs.

```bash
source install/setup.bash
bash run_benchmark.sh
```

An optional argument specifies a runtime buffer (in nanoseconds) to add to each SCHED_DEADLINE runtime parameter:

```bash
bash run_benchmark.sh 500000
```

Results are saved to `benchmark_results/`.

## Usage

### DAG YAML format

Create a YAML file that describes the desired DAG structure. An example is given below.

```yaml
callbacks:
  # DAG0
  - id: 0
    execution_time_ms: 100
    period_ms: 200
  - id: 1
    execution_time_ms: 50
  - id: 2
    execution_time_ms: 30

  # DAG1
  - id: 3
    execution_time_ms: 20
    period_ms: 100

  # DAG2
  - id: 4
    execution_time_ms: 100
    period_ms: 1000

  # DAG3
  - id: 5
    execution_time_ms: 30
    period_ms: 100

communications:
  - from: 0
    to: 1
  - from: 1
    to: 2
```

- Callbacks with `period_ms` become **timers** (DAG roots).
- Callbacks without it become **subscriptions** triggered via `communications` edges.
- When a callback has multiple incoming edges (up to 2), `message_filters::Synchronizer` with `ExactTime` policy is used to fire the callback only when all upstream messages arrive.
- Optional `deadline_ms` enables response-time tracking and deadline-miss statistics printed at shutdown.

### CIE thread configuration

Each benchmark case in `ActualDAGSet/` has a corresponding `cie_case_*.yaml` file that configures per-thread scheduling policy (SCHED_DEADLINE, SCHED_FIFO, etc.) and CPU affinity via `agnocast_cie_thread_configurator`.

### Performance visualization

Execution times are logged to CSV files in `exec_time_logs/`. `evaluate.ipynb` provides simple callback scheduling charts from those CSVs.
