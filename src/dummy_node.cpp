#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "static_callback_isolated_executor.hpp"
#include "yaml-cpp/yaml.h"

#include "std_msgs/msg/string.hpp"

#include <sys/syscall.h>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

const long long MESSAGE_SIZE = 1024;
const std::string DAG_FILE_PATH = "dag.yaml";

class DummyNode : public rclcpp::Node
{
public:
  explicit DummyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sample_node", "/sample_space/sample_subspace", options)
  {
    YAML::Node dag = YAML::LoadFile(DAG_FILE_PATH);
    num_increments_in_1ms_ = count_increment_in_1ms();

    for (const auto & cb : dag["callbacks"]) {
      auto publisher =
        this->create_publisher<std_msgs::msg::String>("topic_" + cb["id"].as<std::string>(), 10);
      publishers_.push_back(publisher);
      auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      groups_.push_back(group);

      if (cb["period_ms"]) {
        // timer
        auto timer = this->create_wall_timer(
          std::chrono::milliseconds(cb["period_ms"].as<int>()),
          [this, cb]() {
            auto message = std_msgs::msg::String();
            message.data = "Hello from timer-callback" + std::to_string(cb["id"].as<int>());
            dummy_work(cb["execution_time_ms"].as<int>());
            publishers_[cb["id"].as<int>()]->publish(std::move(message));
          },
          group);
        timers_.push_back(timer);
      }

      else {
        // subscription
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = group;
        auto subscription = this->create_subscription<std_msgs::msg::String>(
          "topic_" + get_sub_topic_id(dag["communications"], cb["id"].as<int>()), 1,
          [this, cb](const std_msgs::msg::String::SharedPtr msg [[maybe_unused]]) {
            auto message = std_msgs::msg::String();
            message.data = "Hello from subscription-callback" + std::to_string(cb["id"].as<int>());
            dummy_work(cb["execution_time_ms"].as<int>());
            publishers_[cb["id"].as<int>()]->publish(std::move(message));
          },
          sub_options);
        subscriptions_.push_back(subscription);
      }
    }
  }

private:
  long long count_increment_in_1ms()
  {
    auto start = std::chrono::high_resolution_clock::now();
    int count = 0;
    while (
      std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start)
        .count() < 1.0) {
      ++count;
    }

    return count;
  }

  std::string get_sub_topic_id(const YAML::Node & comms, int sub_cb_id)
  {
    for (const auto & comm : comms) {
      if (comm["to"].as<int>() == sub_cb_id) {
        return comm["from"].as<std::string>();
      }
    }
    return "";
  }

  void dummy_work(int execution_time_ms)
  {
    long needed_increments = num_increments_in_1ms_ * execution_time_ms;
    long count = 0;
    for (long i = 0; i < needed_increments; ++i) {
      ++count;
    }
  }

  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> groups_;

  long long num_increments_in_1ms_;
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
