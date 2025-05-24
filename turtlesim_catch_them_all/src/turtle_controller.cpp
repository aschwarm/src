#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include "turtle_interfaces/msg/turtle_array.hpp"
#include "turtle_interfaces/msg/turtle.hpp"
#include "turtle_interfaces/srv/catch_turtle.hpp"

#define PI 3.1415926535f

using namespace std::placeholders;
using namespace std::chrono_literals;

class TurtleControllerNode : public rclcpp::Node
{
    public:
        TurtleControllerNode() : Node("turtle_controller"), current_pose_(nullptr)
        {
            this-> lin_Kp_ = 2.0;
            this-> ang_Kp_ = 8.0;
            
            pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,
            std::bind(&TurtleControllerNode::callbackTurtle1Pose,this,_1));
            
            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

            control_timer_ = this->create_wall_timer(std::chrono::duration<double>(0.1),
                                std::bind(&TurtleControllerNode::control_loop,this));

            turtles_to_catch_subscriber_ = this->create_subscription<turtle_interfaces::msg::TurtleArray>("alive_turtles",10,
            std::bind(&TurtleControllerNode::callbackTurtlesToCatch,this, _1));

            catch_turtle_client_ = this->create_client<turtle_interfaces::srv::CatchTurtle>("catch_turtle", 10);

            RCLCPP_INFO(this->get_logger(),"Turtle contorller has been started");
        }

    private:
    
    // Variables
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    turtlesim::msg::Pose::SharedPtr current_pose_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Subscription<turtle_interfaces::msg::TurtleArray>::SharedPtr turtles_to_catch_subscriber_; 

    rclcpp::Client<turtle_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_client_;

    std::vector<std::shared_ptr<std::thread>> catch_turtle_thread_;
    
    float target_x_;
    float target_y_;

    float lin_Kp_;
    float ang_Kp_;

    turtle_interfaces::msg::Turtle turtle_to_catch_;

    // Methods
    void callbackTurtle1Pose(const turtlesim::msg::Pose::SharedPtr pose)
    {
        this->current_pose_ = pose;
    }

    void control_loop()
    {
        if(this->current_pose_ == nullptr || this->turtle_to_catch_.name == "") return;

        float dist_x = this->turtle_to_catch_.x - this->current_pose_->x;
        float dist_y = this->turtle_to_catch_.y - this->current_pose_->y;

        float distance = std::sqrt(dist_x*dist_x + dist_y * dist_y);

        auto cmd = geometry_msgs::msg::Twist();

        if(distance > 0.5)
        {
            // Position command
            cmd.linear.x = this->lin_Kp_ * distance;

            // Orientation command
            float goal_theta = std::atan2(dist_y, dist_x);
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
           cmd.linear.x = 0.0;
           cmd.angular.z = 0.0;

           catch_turtle_thread_.push_back(std::make_shared<std::thread>(
            std::bind(&TurtleControllerNode::callCatchTurtleService, this, turtle_to_catch_.name)));
           
            turtle_to_catch_.name = "";


        }

        this->cmd_vel_publisher_->publish(cmd);
    }

    float getDistance(turtle_interfaces::msg::Turtle turtle)
    {
        float dist_x = turtle.x - this->current_pose_->x;
        float dist_y = turtle.y - this->current_pose_->y;

        float distance = std::sqrt(dist_x*dist_x + dist_y * dist_y);

        return distance;
    }
    
    void callbackTurtlesToCatch(const turtle_interfaces::msg::TurtleArray::SharedPtr turtle_list)
    {
        if(! turtle_list->turtles.empty())
        {
            
            turtle_interfaces::msg::Turtle closest_turtle = turtle_list->turtles.at(0);
            float closest_turtle_distance = getDistance(closest_turtle);
            
            for (int i = 1; i < (int)turtle_list->turtles.size(); i++)
            {
                float distance = getDistance(turtle_list->turtles.at(i));
                if(distance < closest_turtle_distance)
                {
                    closest_turtle = turtle_list->turtles.at(i);
                    closest_turtle_distance = distance;
                }
            }

            this->turtle_to_catch_ = closest_turtle;
        }
    }

    void callCatchTurtleService(std::string turtle_name)
    {
        while(! catch_turtle_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Catch server...");
        }

        auto request = std::make_shared<turtle_interfaces::srv::CatchTurtle::Request>();
        request->name = turtle_name;

        auto future = catch_turtle_client_->async_send_request(request);

         try
        {
            auto response = future.get();

            if(!response->success)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to catch turtle: %s", turtle_name.c_str());
            }
            
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Catch Service call failed");
        }

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