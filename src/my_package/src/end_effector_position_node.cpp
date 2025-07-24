#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class EndEffectorPositionNode : public rclcpp::Node
{
public:
    EndEffectorPositionNode()
    : Node("end_effector_position_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/end_effector_position", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&EndEffectorPositionNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        try
        {
            transformStamped = tf_buffer_.lookupTransform(
                "lbr_link_0", "lbr_link_7", tf2::TimePointZero);

            auto & t = transformStamped.transform.translation;

            //RCLCPP_INFO(this->get_logger(),
                        //"Effector position: x=%.4f, y=%.4f, z=%.4f", t.x, t.y, t.z);

            geometry_msgs::msg::PointStamped point_msg;
            point_msg.header = transformStamped.header;
            point_msg.point.x = t.x;
            point_msg.point.y = t.y;
            point_msg.point.z = t.z;

            publisher_->publish(point_msg);
        }
        catch (tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EndEffectorPositionNode>());
    rclcpp::shutdown();
    return 0;
}