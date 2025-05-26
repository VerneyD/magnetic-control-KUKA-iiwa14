#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class CommandListener : public 
rclcpp::Node
{
public:
  CommandListener()
  : Node("command_listener")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/lbr/servo_node/delta_twist_cmds", 10, std::bind(&CommandListener::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    static int counter = 0 ; 
    counter ++ ;
    if (counter < 30) return;
        if (msg->twist.linear.x == 0 and msg->twist.linear.y == 0 and msg->twist.linear.z == 0 and msg->twist.angular.x == 0 and msg->twist.angular.y == 0 and msg->twist.angular.z == 0 ){
            RCLCPP_INFO(this->get_logger(), "Waiting for movement. \n \
            Left horizontal joystick controls the x-axis translation. Hold LB to control the x-axis rotation. \n \
            Left vertical joystick controls the y-axis translation.  Hold LB to control the y-axis rotation. \n\
            Right vertical joystick controls the z-axis translation.  Hold LB to control the z-axis rotation. \n");
            counter = 0 ;
        }

        else {
            RCLCPP_INFO(this->get_logger(), "Effector movement : \n, \
            X-axis linear velocity :'%f'\n, \
            Y-axis linear velocity :'%f'\n , \
            Z-axis linear velocity :'%f'\n, \
            X-axis angular velocity :' %f'\n, \
            Y-axis angular velocity :'%f'\n, \
            Z-axis angular velocity :'%f' \n", 
            msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
            msg->twist.angular.x,msg->twist.angular.y,msg->twist.angular.z);
            counter = 0 ;
    } 

    
    }
    

    
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandListener>());
  rclcpp::shutdown();
  return 0;
}