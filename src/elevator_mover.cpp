#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using SUBTYPE = sensor_msgs::msg::JointState;
using PUBTYPE = std_msgs::msg::Float64MultiArray;

const std::string TOPIC_1 = "/joint_states";
const std::string TOPIC_2 = "/joint_position_controller/commands";

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("elevator_mover_node") {

    subscription_ = this->create_subscription<SUBTYPE>(
      TOPIC_1, 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    publisher_ = this->create_publisher<PUBTYPE>(
      TOPIC_2, 10);

    RCLCPP_INFO(this->get_logger(), "Elevator Mover Node Started.");
  }

private:
  void topic_callback(const SUBTYPE::SharedPtr msg) {

    // Find the index of "elevator_joint"
    auto it = std::find(msg->name.begin(), msg->name.end(), "elevator_joint");
    if (it == msg->name.end()) {
      RCLCPP_WARN(this->get_logger(), "elevator_joint not found in joint_states!");
      return;
    }

    size_t index = std::distance(msg->name.begin(), it);
    double elevator_position = msg->position[index];

    // Publish the position to the controller
    auto message = PUBTYPE();
    message.data.push_back(elevator_position);
    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Published Elevator Position: %f", elevator_position);
  }

  rclcpp::Subscription<SUBTYPE>::SharedPtr subscription_;
  rclcpp::Publisher<PUBTYPE>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

