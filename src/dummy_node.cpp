#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "agnocast/agnocast_callback_isolated_executor.hpp"
#include "yaml-cpp/yaml.h"

#include "std_msgs/msg/header.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <pthread.h>
#include <sys/syscall.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <vector>

using namespace std::chrono_literals;
using namespace std::chrono;

using HeaderMsg = std_msgs::msg::Header;

namespace message_filters {
namespace message_traits {
template <>
struct TimeStamp<HeaderMsg, void> {
  static rclcpp::Time value(const HeaderMsg & m)
  {
    return rclcpp::Time(m.stamp.sec, m.stamp.nanosec, RCL_ROS_TIME);
  }
};
}  // namespace message_traits
}  // namespace message_filters

const std::string DAG_FILE_PATH = "dags.yaml";
const std::string EXEC_TIME_LOGS_DIR = "exec_time_logs";

builtin_interfaces::msg::Time us_to_time(int64_t us)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(us / 1000000);
  t.nanosec = static_cast<uint32_t>((us % 1000000) * 1000);
  return t;
}

int64_t time_to_us(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000 + t.nanosec / 1000;
}

class DummyNode : public rclcpp::Node
{
  using ExactPolicy2 = message_filters::sync_policies::ExactTime<HeaderMsg, HeaderMsg>;
  using Sync2 = message_filters::Synchronizer<ExactPolicy2>;

public:
  explicit DummyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sample_node", "/sample_space/sample_subspace", options)
  {
    // Setup
    YAML::Node dag = YAML::LoadFile(DAG_FILE_PATH);
    if (!std::filesystem::exists(EXEC_TIME_LOGS_DIR)) {
      std::filesystem::create_directory(EXEC_TIME_LOGS_DIR);
    }
    num_increments_in_1ms_ = count_increment_in_1ms();

    // Build incoming edges map
    std::map<int, std::vector<int>> incoming_edges;
    for (const auto & comm : dag["communications"]) {
      incoming_edges[comm["to"].as<int>()].push_back(comm["from"].as<int>());
    }

    // Create DAGs
    for (const auto & cb : dag["callbacks"]) {
      auto file = std::make_shared<std::ofstream>(
        EXEC_TIME_LOGS_DIR + "/callback" + cb["id"].as<std::string>() + ".csv");
      *file << "scheduling_policy,time_ns,response_time_us" << std::endl;
      files_.push_back(file);
      publishers_.emplace_back(
        this->create_publisher<HeaderMsg>("topic_" + cb["id"].as<std::string>(), 10));
      auto group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      groups_.push_back(group);

      // Parse deadline_ms if present
      if (cb["deadline_ms"]) {
        deadline_ms_[cb["id"].as<int>()] = cb["deadline_ms"].as<int>();
      }

      // Create timer or subscription
      check_overflow(cb["execution_time_ms"].as<unsigned long long>());
      if (cb["period_ms"]) {
        // timer
        timers_.emplace_back(this->create_wall_timer(
          milliseconds(cb["period_ms"].as<int>()),
          [this, cb]() {
            int cb_id = cb["id"].as<int>();
            auto start = high_resolution_clock::now();
            int64_t chain_start_us =
              duration_cast<microseconds>(start.time_since_epoch()).count();
            write_start(cb_id, start);
            dummy_work(cb["execution_time_ms"].as<unsigned long long>());
            publish(cb_id, chain_start_us);
            auto finish = high_resolution_clock::now();
            write_finish(cb_id, start, finish);
            if (deadline_ms_.count(cb_id)) {
              int64_t finish_us =
                duration_cast<microseconds>(finish.time_since_epoch()).count();
              response_times_[cb_id].push_back(finish_us - chain_start_us);
            }
          },
          group));
      } else {
        int cb_id = cb["id"].as<int>();
        const auto & sources = incoming_edges[cb_id];

        if (sources.size() == 1) {
          // Single incoming edge: normal subscription
          rclcpp::SubscriptionOptions sub_options;
          sub_options.callback_group = group;
          subscriptions_.emplace_back(this->create_subscription<HeaderMsg>(
            "topic_" + std::to_string(sources[0]), 1,
            [this, cb](const HeaderMsg::SharedPtr msg) {
              int cb_id = cb["id"].as<int>();
              int64_t chain_start_us = time_to_us(msg->stamp);
              auto start = high_resolution_clock::now();
              write_start(cb_id, start);
              dummy_work(cb["execution_time_ms"].as<unsigned long long>());
              publish(cb_id, chain_start_us);
              auto finish = high_resolution_clock::now();
              write_finish(cb_id, start, finish);
              if (deadline_ms_.count(cb_id)) {
                int64_t finish_us =
                  duration_cast<microseconds>(finish.time_since_epoch()).count();
                response_times_[cb_id].push_back(finish_us - chain_start_us);
              }
            },
            sub_options));
        } else if (sources.size() == 2) {
          // Two incoming edges: use message_filters Synchronizer with ExactTime
          rclcpp::SubscriptionOptions sub_options0;
          sub_options0.callback_group = group;
          auto group1 =
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
          groups_.push_back(group1);
          rclcpp::SubscriptionOptions sub_options1;
          sub_options1.callback_group = group1;

          auto mf_sub0 = std::make_shared<message_filters::Subscriber<HeaderMsg>>(
            this, "topic_" + std::to_string(sources[0]), rmw_qos_profile_default, sub_options0);
          auto mf_sub1 = std::make_shared<message_filters::Subscriber<HeaderMsg>>(
            this, "topic_" + std::to_string(sources[1]), rmw_qos_profile_default, sub_options1);

          auto sync = std::make_shared<Sync2>(ExactPolicy2(10), *mf_sub0, *mf_sub1);

          auto callback_fn = [this, cb](HeaderMsg::ConstSharedPtr msg0, HeaderMsg::ConstSharedPtr) {
            int cb_id = cb["id"].as<int>();
            int64_t chain_start_us = time_to_us(msg0->stamp);
            auto start = high_resolution_clock::now();
            write_start(cb_id, start);
            dummy_work(cb["execution_time_ms"].as<unsigned long long>());
            publish(cb_id, chain_start_us);
            auto finish = high_resolution_clock::now();
            write_finish(cb_id, start, finish);
            if (deadline_ms_.count(cb_id)) {
              int64_t finish_us =
                duration_cast<microseconds>(finish.time_since_epoch()).count();
              response_times_[cb_id].push_back(finish_us - chain_start_us);
            }
          };

          using NullP = const std::shared_ptr<message_filters::NullType const> &;
          using HdrP = const HeaderMsg::ConstSharedPtr &;
          std::function<void(HdrP, HdrP, NullP, NullP, NullP, NullP, NullP, NullP, NullP)>
            wrapped_cb = std::bind(
              callback_fn, std::placeholders::_1, std::placeholders::_2);
          sync->registerCallback(wrapped_cb);

          mf_subscribers_.push_back(mf_sub0);
          mf_subscribers_.push_back(mf_sub1);
          synchronizers_.push_back(sync);
        } else {
          RCLCPP_ERROR(
            this->get_logger(), "Callback %d has %zu incoming edges (only 1 or 2 supported)",
            cb_id, sources.size());
          rclcpp::shutdown();
        }
      }
    }
  }

  ~DummyNode() override { print_response_time_statistics(); }

private:
  unsigned long long count_increment_in_1ms()
  {
    auto start = high_resolution_clock::now();
    volatile unsigned long long count = 0;
    while (duration<double, std::nano>(high_resolution_clock::now() - start).count() < 987000) {
      ++count;
    }

    const int ADJUST_FACTOR = 13;  // FIXME
    return count * ADJUST_FACTOR;
  }

  void check_overflow(unsigned long long execution_time_ms)
  {
    if (
      execution_time_ms > std::numeric_limits<unsigned long long>::max() / num_increments_in_1ms_) {
      RCLCPP_ERROR(this->get_logger(), "Overflow detected. Please decrease the execution time.");
      rclcpp::shutdown();
    }
  }

  void dummy_work(unsigned long long execution_time_ms)
  {
    unsigned long long needed_increments = num_increments_in_1ms_ * execution_time_ms;
    volatile unsigned long long count = 0;
    for (unsigned long long i = 0; i < needed_increments; ++i) {
      ++count;
    }
  }

  std::string get_policy()
  {
    int self_policy;
    sched_param self_param;
    pthread_getschedparam(pthread_self(), &self_policy, &self_param);
    std::string scheduling_policy;
    switch (self_policy) {
      case SCHED_OTHER:
        scheduling_policy = "SCHED_OTHER";
        break;
      case SCHED_FIFO:
        scheduling_policy = "SCHED_FIFO";
        break;
      case SCHED_RR:
        scheduling_policy = "SCHED_RR";
        break;
      case SCHED_DEADLINE:
        scheduling_policy = "SCHED_DEADLINE";
        break;
      default:
        scheduling_policy = "UNKNOWN";
        break;
    }

    return scheduling_policy;
  }

  void write_start(int cb_id, _V2::system_clock::time_point start)
  {
    *files_[cb_id] << get_policy() << ","
                   << duration_cast<microseconds>(start.time_since_epoch()).count() << ","
                   << "0" << std::endl;
  }

  void write_finish(
    int cb_id, _V2::system_clock::time_point start, _V2::system_clock::time_point finish)
  {
    *files_[cb_id] << get_policy() << ","
                   << duration_cast<microseconds>(finish.time_since_epoch()).count() << ","
                   << duration_cast<microseconds>(finish - start).count() << std::endl;
  }

  void publish(int cb_id, int64_t chain_start_us)
  {
    auto message = HeaderMsg();
    message.stamp = us_to_time(chain_start_us);
    publishers_[cb_id]->publish(std::move(message));
  }

  void print_response_time_statistics()
  {
    if (deadline_ms_.empty()) {
      return;
    }

    std::cerr << "\n=== Response Time Statistics ===" << std::endl;
    for (const auto & [cb_id, deadline] : deadline_ms_) {
      auto it = response_times_.find(cb_id);
      if (it == response_times_.end() || it->second.empty()) {
        std::cerr << "Task " << cb_id << " (deadline: " << deadline
                  << " ms): no data" << std::endl;
        continue;
      }

      const auto & rts = it->second;
      size_t count = rts.size();

      double sum = std::accumulate(rts.begin(), rts.end(), 0.0);
      double mean = sum / count;

      int64_t min_val = *std::min_element(rts.begin(), rts.end());
      int64_t max_val = *std::max_element(rts.begin(), rts.end());

      double sq_sum = 0.0;
      for (int64_t v : rts) {
        double diff = v - mean;
        sq_sum += diff * diff;
      }
      double stddev = std::sqrt(sq_sum / count);

      int64_t deadline_us = static_cast<int64_t>(deadline) * 1000;
      size_t miss_count = 0;
      for (int64_t v : rts) {
        if (v > deadline_us) {
          ++miss_count;
        }
      }

      std::cerr << "Task " << cb_id << " (deadline: " << deadline << " ms):" << std::endl;
      std::cerr << "  count:  " << count << std::endl;
      std::cerr << "  mean:   " << std::fixed << std::setprecision(1) << mean / 1000.0
                << " ms" << std::endl;
      std::cerr << "  min:    " << min_val / 1000.0 << " ms" << std::endl;
      std::cerr << "  max:    " << max_val / 1000.0 << " ms" << std::endl;
      std::cerr << "  stddev: " << stddev / 1000.0 << " ms" << std::endl;
      std::cerr << "  deadline misses: " << miss_count << " / " << count << " ("
                << std::fixed << std::setprecision(1)
                << (100.0 * miss_count / count) << "%)" << std::endl;
    }
  }

  std::vector<rclcpp::Publisher<HeaderMsg>::SharedPtr> publishers_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::vector<rclcpp::Subscription<HeaderMsg>::SharedPtr> subscriptions_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> groups_;
  std::vector<std::shared_ptr<std::ofstream>> files_;

  std::vector<std::shared_ptr<message_filters::Subscriber<HeaderMsg>>> mf_subscribers_;
  std::vector<std::shared_ptr<Sync2>> synchronizers_;

  unsigned long long num_increments_in_1ms_;
  std::map<int, int> deadline_ms_;
  std::map<int, std::vector<int64_t>> response_times_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(DummyNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DummyNode>();
  auto executor = std::make_shared<agnocast::CallbackIsolatedAgnocastExecutor>();
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
