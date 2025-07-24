#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <control_msgs/msg/joint_jog.hpp>

class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher()
        : Node("command_publisher")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&CommandPublisher::joy_callback, this, std::placeholders::_1));  
            // subscription to the controller's state: this will allow us to get the position of the joysticks
            // It is a message of type sensor_msgs/msg/joy on the topic  /joy

        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/lbr/servo_node/delta_twist_cmds", 10);
            // this programm will publish a command in velocity. It needs to be a message of type
            // geometry_msgs/msg/twist_stamped on the topic /servo_node/delta_twist_cmds
        
        publisher_2 = this->create_publisher<control_msgs::msg::JointJog>(
            "/lbr/servo_node/delta_joint_cmds", 10);


        scale_linear_ = 0.4; // maximum linear velocity (found in .yaml file)
        scale_angular_ = 0.8; // maximum angular velocity (found in .yaml file)

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&CommandPublisher::publish_command, this)); //the message will be published at a 30Hz rate 
            
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        last_joy_msg_ = msg; //renaming the message from the joystick for clarity purpose
    }

    void publish_command()
    {
        if (!last_joy_msg_) return; // if we get some data from the controller

        auto twist_msg = geometry_msgs::msg::TwistStamped(); // we create the message that we will publish (type geometry_msgs/msg/twist_stamped)
        twist_msg.header.stamp = this->get_clock()->now();
        twist_msg.header.frame_id = "lbr_link_0"; // the base frame for the velocity's value

        auto joint_msg = control_msgs::msg::JointJog();    
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.header.frame_id = "lbr_link_0";


    // Define the XBox buttons 
        float LT = last_joy_msg_->axes[2]; // LT
        float RT = last_joy_msg_->axes[5]; // RT
        float RSH = last_joy_msg_->axes[3]; // right stick horizontal
        float RSV = last_joy_msg_->axes[4]; // right stick vertical
        float LSH = last_joy_msg_->axes[0]; // left stick horizontal
        float LSV = last_joy_msg_->axes[1]; // left stick vertical
        float DPadH = last_joy_msg_->axes[6]; // DPad horizontal
        float LB = last_joy_msg_->buttons[4]; // LB
        float RB = last_joy_msg_->buttons[5]; // RB
        float X = last_joy_msg_->buttons[2]; // X


        
        // Speed of the robot 
        double speed = 1.0 + 1.5 * (1.0f-RT)/2.0f - 0.5* (1.0f-LT)/2.0f;

        // Optionnel : limite la valeur de speed
        if (speed < 0.0) speed = 0.0;
        if (speed > 3.0) speed = 3.0;

        if (X == 1 ){
            speed = 1.0;
        }

        twist_msg.twist.linear.x = -1* speed *scale_linear_ * RSH; 
        twist_msg.twist.linear.y = speed *scale_linear_ * RSV;
        
        twist_msg.twist.angular.x = speed *scale_angular_ * LSH;
        twist_msg.twist.angular.y = speed *scale_angular_ * LSV;
        twist_msg.twist.angular.z = speed *scale_angular_ * DPadH;

            // LB diminue le mouvement linéaire sur l'axe Z, RB l'augmente
    if (LB) {
        twist_msg.twist.linear.z = -speed * scale_linear_;
    } else if (RB) {
        twist_msg.twist.linear.z = speed * scale_linear_;
    }
            
        publisher_->publish(twist_msg);
        

        
    }
    


    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr publisher_2;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;  
    double scale_linear_;
    double scale_angular_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandPublisher>());
    rclcpp::shutdown();
    return 0;
}