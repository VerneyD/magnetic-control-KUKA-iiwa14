#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStatePublisher : public rclcpp::Node
{
public:
  JointStatePublisher()
  : Node("joint_state_publisher_cpp")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10));
    timer_ = this->create_wall_timer(
      100ms, std::bind(&JointStatePublisher::on_timer, this));
    start_time_ = this->now();

    // Noms des joints
    for (int i = 0; i < 8; ++i) {
      joint_names_.push_back("lbr_joint_" + std::to_string(i));
    }
  }

private:
  void on_timer()
  {
    auto msg = sensor_msgs::msg::JointState();
    auto now = this->now();
    double elapsed = (now - start_time_).seconds();

    msg.header.stamp = now;
    msg.name = joint_names_;

    // Positions en sinusoïdes décalées
    msg.position.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      msg.position[i] = 0.5 * std::sin(elapsed + double(i));
    }

    // Velocités et efforts à zéro
    msg.velocity.assign(joint_names_.size(), 0.0);
    msg.effort.assign(joint_names_.size(), 0.0);

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published %zu joint positions", msg.position.size());
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
