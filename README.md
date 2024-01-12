# ROS 2 Task Set

## Setup

1. Follow [the steps on this page](https://tier4.atlassian.net/wiki/spaces/~5ed0b4584824b20c18371c06/pages/2886861884/CallbackGroup#step1%3A-%E3%81%9D%E3%82%8C%E3%81%9E%E3%82%8C%E3%83%93%E3%83%AB%E3%83%89%26%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB) to install dependent libraries (TIER IV INTERNAL).
2. `cd ros2_task_set`
3. `colcon build`

## Usage

1. Create a yaml file that describes the desired DAG structure. An example is given below.

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

> [!WARNING]
> Complex trigger semantics such as Sync of multiple messages are not yet supported.

2. Refer to [this document](https://tier4.atlassian.net/wiki/spaces/~5ed0b4584824b20c18371c06/pages/2886861884/CallbackGroup#step3%3A-%E8%A8%AD%E5%AE%9A%E3%83%95%E3%82%A1%E3%82%A4%E3%83%AB%E3%81%AE%E9%9B%9B%E5%BD%A2%E3%82%92%E4%BD%9C%E6%88%90%E3%83%BB%E7%B7%A8%E9%9B%86) to set each thread's scheduling policy and affinity (TIER IV INTERNAL).
3. Performance can be measured and visualized using tools such as [CARET](https://tier4.github.io/caret_doc/latest/). If you just want to draw a simple callback scheduling chart, `evaluate.ipynb` offers this.
