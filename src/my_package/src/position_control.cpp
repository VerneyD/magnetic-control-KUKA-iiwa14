#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <cmath>

class PositionControl : public rclcpp::Node
{
public:
    PositionControl()
        : Node("command_publisher"), target_reached(false), speed_initialized(false)
    {
        // Subscribe to current end-effector position
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/end_effector_position", 10,
            std::bind(&PositionControl::msg_callback, this, std::placeholders::_1));

        // Publisher for velocity commands
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/lbr/servo_node/delta_twist_cmds", 10);

        // Timer to publish velocity commands at ~30 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 Hz
            std::bind(&PositionControl::publish_command, this));

        // Target position
        final_position.x = 0.48;
        final_position.y = 0.40;
        final_position.z = 0.33;

        frequency = 30.0;  // 30 Hz
    }

private:
    void msg_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        current_position = msg->point;

        RCLCPP_INFO(this->get_logger(), "Received position: x=%f, y=%f, z=%f",
                    current_position.x, current_position.y, current_position.z);

        // Initialize speeds after first position is received
        if (!speed_initialized)
        {
            double dx = final_position.x - current_position.x;
            double dy = final_position.y - current_position.y;
            double dz = final_position.z - current_position.z;

            speed_x =  std::copysign(0.5, dx);
            speed_y =  std::copysign(0.5, dy);
            speed_z =  std::copysign(0.5, dz);

            double time_x = frequency * dx ;
            double time_y = frequency * dy ; 
            double time_z = frequency * dz ; 
            movement_time = std::max(std::max(time_x, time_y), time_z);


            speed_initialized = true;

            RCLCPP_INFO(this->get_logger(), "Initialized speeds: vx=%f, vy=%f, vz=%f",
                        speed_x, speed_y, speed_z);
            RCLCPP_INFO(this->get_logger(), "Time : tx=%f, ty=%f, tz=%f ,t=%f",
                        time_x, time_y, time_z, movement_time);
        }
    }

    void publish_command()
{
    if (!speed_initialized || target_reached)
    {
        return; // Don't publish until speed is initialized or if already reached
    }

    double dx = final_position.x - current_position.x;
    double dy = final_position.y - current_position.y;
    double dz = final_position.z - current_position.z;

    double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
    RCLCPP_INFO(this->get_logger(), "Distance to target: %f", norm);

    // If close to the full target, stop everything
    if (norm < 0.001)
    {
        target_reached = true;

        auto stop_msg = geometry_msgs::msg::TwistStamped();
        stop_msg.header.stamp = this->get_clock()->now();
        stop_msg.header.frame_id = "lbr_link_0";
        publisher_->publish(stop_msg);

        RCLCPP_INFO(this->get_logger(), "Target reached. Stopping.");
        return;
    }

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->get_clock()->now();
    twist_msg.header.frame_id = "lbr_link_0";

    // Precision threshold for each axis
    const double axis_threshold = 0.001;

    twist_msg.twist.linear.x = (std::abs(dx) < axis_threshold) ? 0.0 : speed_x;
    twist_msg.twist.linear.y = (std::abs(dy) < axis_threshold) ? 0.0 : speed_y;
    twist_msg.twist.linear.z = (std::abs(dz) < axis_threshold) ? 0.0 : speed_z;

    publisher_->publish(twist_msg);
}


    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Position and control data
    geometry_msgs::msg::Point current_position;
    geometry_msgs::msg::Point final_position;

    // Motion control
    double movement_time;   // Movement duration in seconds
    double frequency;       // Control frequency (Hz)
    double speed_x, speed_y, speed_z;

    bool target_reached;
    bool speed_initialized;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionControl>());
    rclcpp::shutdown();
    return 0;
}
