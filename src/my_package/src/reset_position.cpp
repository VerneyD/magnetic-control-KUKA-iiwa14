#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "math.h"


class TrajectoryCommander : public rclcpp::Node
{
public:
    TrajectoryCommander()
        : Node("trajectory_commander"), publish_count_(0)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/lbr/joint_trajectory_controller/joint_trajectory", 10);

        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/lbr/joint_states", 10, std::bind(&TrajectoryCommander::topic_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),  // Publier toutes les 500 ms
            std::bind(&TrajectoryCommander::publish_trajectory, this));
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
    if (msg->position.size() == 7 && initial_positions.empty()) {
        initial_positions = msg->position;
        RCLCPP_INFO(this->get_logger(), "Initial joint states received.");

        // Now start the timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TrajectoryCommander::publish_trajectory, this)
        );
    }
}

    
    void publish_trajectory()
    {
        if (publish_count_ >=1) {
            RCLCPP_INFO(this->get_logger(), "Done sending trajectory.");
            timer_->cancel();  // Stop après 10 publications
            return;
        }

        trajectory_msgs::msg::JointTrajectory msg;

        msg.joint_names = {
            "lbr_A1", "lbr_A2", "lbr_A3",
            "lbr_A4", "lbr_A5", "lbr_A6", "lbr_A7"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        point.positions = {
            -1.57, -0.663, 0.0,
            1.309, 0.0, -1.169, 0.0
        };
        final_positions = point.positions;

        // Calculate differences between each positions
        difference.resize(7);

        double max_difference = fabs(final_positions[0] - initial_positions[0]); 
        for (int i=1; i<7;i++){
            difference[i] = fabs(final_positions[i] - initial_positions[i]);
            if (difference[i] > max_difference){
                max_difference = difference[i] ;
            }
        }
        
        float time = max_difference/max_speed ; 
        if (time < 5){
            time = 5.0;
        }

        // Calculer le temps écoulé
        point.time_from_start = rclcpp::Duration::from_seconds(time);  

        msg.points.push_back(point);

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Trajectory command sent (publication #%d).", publish_count_ + 1);

        publish_count_++;
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    int publish_count_;
    std::vector<double> initial_positions;
    std::vector<double> final_positions;
    std::vector<double> difference;
    float max_speed = 80*M_PI/180 ; 

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryCommander>());
    rclcpp::shutdown();
    return 0;
}
