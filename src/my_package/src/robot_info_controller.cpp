#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
#include <mutex>

class InfoListener : public rclcpp::Node
{
public:
    InfoListener()
        : Node("info_listener")
    {
        joints_subscriber_2 = this->create_subscription<sensor_msgs::msg::JointState>(
            "/lbr/joint_states", 10,
            std::bind(&InfoListener::topic_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // 2 Hz = 500ms
            std::bind(&InfoListener::timer_callback, this));
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_positions_ = msg->position;
    }

    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (latest_positions_.size() < 7)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for at least 7 joint positions...");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Joint values:\n"
            "Joint 1 = %.2f\n"
            "Joint 2 = %.2f\n"
            "Joint 3 = %.2f\n"
            "Joint 4 = %.2f\n"
            "Joint 5 = %.2f\n"
            "Joint 6 = %.2f\n"
            "Joint 7 = %.2f\n",
            latest_positions_[1] * 180.0 / M_PI,
            latest_positions_[2] * 180.0 / M_PI,
            latest_positions_[3] * 180.0 / M_PI,
            latest_positions_[5] * 180.0 / M_PI,
            latest_positions_[0] * 180.0 / M_PI,
            latest_positions_[4] * 180.0 / M_PI,
            latest_positions_[6] * 180.0 / M_PI);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_subscriber_2;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> latest_positions_;
    std::mutex mutex_;  // Protect shared data
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InfoListener>());
    rclcpp::shutdown();
    return 0;
}
