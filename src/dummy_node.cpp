#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "static_callback_isolated_executor.hpp"
#include "yaml-cpp/yaml.h"

#include "std_msgs/msg/string.hpp"

#include <sys/syscall.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>

using namespace std::chrono_literals;
using namespace std::chrono;

const std::string DAG_FILE_PATH = "dags.yaml";
const std::string EXEC_TIME_LOGS_DIR = "exec_time_logs";

class DummyNode : public rclcpp::Node
{
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

    // Create DAGs
    for (const auto & cb : dag["callbacks"]) {
      auto file = std::make_shared<std::ofstream>(
        EXEC_TIME_LOGS_DIR + "/callback" + cb["id"].as<std::string>() + ".csv");
      *file << "start_time_ms,execution_time_ms" << std::endl;
      files_.push_back(file);
      publishers_.emplace_back(
        this->create_publisher<std_msgs::msg::String>("topic_" + cb["id"].as<std::string>(), 10));
      auto group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      groups_.push_back(group);

      // Create timer or subscription
      if (cb["period_ms"]) {
        // timer
        timers_.emplace_back(this->create_wall_timer(
          milliseconds(cb["period_ms"].as<int>()),
          [this, cb]() {
            auto start = high_resolution_clock::now();
            dummy_work(cb["execution_time_ms"].as<unsigned long long>());
            publish(cb["id"].as<int>());
            write_exec_time(cb["id"].as<int>(), start);
          },
          group));
      }

      else {
        // subscription
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = group;
        subscriptions_.emplace_back(this->create_subscription<std_msgs::msg::String>(
          "topic_" + get_sub_topic_id(dag["communications"], cb["id"].as<int>()), 1,
          [this, cb](const std_msgs::msg::String::SharedPtr msg [[maybe_unused]]) {
            auto start = high_resolution_clock::now();
            dummy_work(cb["execution_time_ms"].as<unsigned long long>());
            publish(cb["id"].as<int>());
            write_exec_time(cb["id"].as<int>(), start);
          },
          sub_options));
      }
    }
  }

private:
  unsigned long long count_increment_in_1ms()
  {
    auto start = high_resolution_clock::now();
    volatile unsigned long long count = 0;
    while (duration<double, std::milli>(high_resolution_clock::now() - start).count() < 1.0) {
      ++count;
    }

    const int ADJUST_FACTOR = 15;  // FIXME
    return count * ADJUST_FACTOR;
  }

  std::string get_sub_topic_id(const YAML::Node & comms, int sub_cb_id)
  {
    for (const auto & comm : comms) {
      if (comm["to"].as<int>() == sub_cb_id) {
        return comm["from"].as<std::string>();
      }
    }

    RCLCPP_ERROR(this->get_logger(), "No subscribe topic found for callback %d", sub_cb_id);
    return "";
  }

  void dummy_work(unsigned long long execution_time_ms)
  {
    unsigned long long needed_increments = num_increments_in_1ms_ * execution_time_ms;
    volatile unsigned long long count = 0;
    for (unsigned long long i = 0; i < needed_increments; ++i) {
      ++count;
    }
  }

  void write_exec_time(int cb_id, _V2::system_clock::time_point start)
  {
    *files_[cb_id] << duration_cast<milliseconds>(start.time_since_epoch()).count() << ","
                   << duration_cast<milliseconds>(high_resolution_clock::now() - start).count()
                   << std::endl;
  }

  void publish(int cb_id)
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello from callback" + std::to_string(cb_id);
    publishers_[cb_id]->publish(std::move(message));
  }

  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> groups_;
  std::vector<std::shared_ptr<std::ofstream>> files_;

  unsigned long long num_increments_in_1ms_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(DummyNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DummyNode>();
  auto executor = std::make_shared<StaticCallbackIsolatedExecutor>();
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
