#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

#define PI 3.1415926535f

using namespace std::placeholders;

class TurtleControllerNode : public rclcpp::Node
{
    public:
        TurtleControllerNode() : Node("turtle_controller")
        {
            this->target_x_ = 8.5;
            this->target_y_ = 7.5;
            this->current_pose_ = nullptr;
            this-> lin_Kp_ = 1.0;
            this-> ang_Kp_ = 1.0;
            
            pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,
            std::bind(&TurtleControllerNode::callbackTurtle1Pose,this,_1));
            
            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

            control_timer_ = this->create_wall_timer(std::chrono::duration<double>(0.1),
                                std::bind(&TurtleControllerNode::control_loop,this));

            RCLCPP_INFO(this->get_logger(),"Turtle contorller has been started");
        }

    private:
    
    // Variables
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    turtlesim::msg::Pose::SharedPtr current_pose_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    float target_x_;
    float target_y_;

    float lin_Kp_;
    float ang_Kp_;

    // Methods
    void callbackTurtle1Pose(const turtlesim::msg::Pose::SharedPtr pose)
    {
        this->current_pose_ = pose;
    }

    void control_loop()
    {
        if(this->current_pose_ == nullptr) return;

        float dist_x = this->target_x_ - this->current_pose_->x;
        float dist_y = this->target_y_ - this->current_pose_->y;

        float distance = sqrt(dist_x*dist_x + dist_y * dist_y);

        auto cmd = geometry_msgs::msg::Twist();

        if(distance > 0.5)
        {
            // Position command
            cmd.linear.x = this->lin_Kp_ * distance;

            
            // Orientation command
            float goal_theta = atan2(dist_y, dist_x);
            float diff_angle = goal_theta - this->current_pose_->theta;
            
            // Normalize the angle
            if(diff_angle > PI)
            {
                diff_angle -= 2 * PI;
            }
            else if(diff_angle < -PI)
            {
                diff_angle += 2 * PI;
            }
            
            cmd.angular.z = this->ang_Kp_ * diff_angle;
            
        }
        else
        {
           // Call catch turtle service

           cmd.linear.x = 0.0;
           cmd.angular.z = 0.0;
        }

        this->cmd_vel_publisher_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}