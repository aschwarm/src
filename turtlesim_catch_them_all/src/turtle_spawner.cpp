#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtle_interfaces/msg/turtle.hpp"
#include "turtle_interfaces/msg/turtle_array.hpp"
#include "turtle_interfaces/srv/catch_turtle.hpp"

#define PI 3.1415926535f

using namespace std::chrono_literals;
using namespace std::placeholders;

class TurtleSpawnerNode : public rclcpp::Node
{
    public:
        TurtleSpawnerNode() : Node("turtle_spawner"), turtleCounter(0)
        {
            this->turtlePrefix = "target";
            
            spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
            spawn_timer_ = this->create_wall_timer(std::chrono::duration<double>(3.0), 
                            std::bind(&TurtleSpawnerNode::spawnNewTurtle, this));

            alive_turtle_publisher_ = this->create_publisher<turtle_interfaces::msg::TurtleArray>("alive_turtles",10);

            catch_turtle_service_ = this->create_service<turtle_interfaces::srv::CatchTurtle>("catch_turtle",
            std::bind(&TurtleSpawnerNode::callbackCatchTurtle, this, _1, _2));
            
            kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        }

    private:

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;

    rclcpp::Publisher<turtle_interfaces::msg::TurtleArray>::SharedPtr alive_turtle_publisher_;

    std::string turtlePrefix;
    int turtleCounter;

    std::vector<turtle_interfaces::msg::Turtle> alive_turtles_;

    std::vector<std::shared_ptr<std::thread>> spawn_turtle_threads_;
    std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;

    rclcpp::Service<turtle_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_service_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;

    void spawnNewTurtle()
    {
        this->turtleCounter++;
        std::string name = this->turtlePrefix + std::to_string(this->turtleCounter);
        float x = (float)rand() / (float)RAND_MAX * 11.0f;
        float y = (float)rand() / (float)RAND_MAX * 11.0f;
        float theta = (float)rand() / (float)RAND_MAX * (2.0f*PI);

        spawn_turtle_threads_.push_back(std::make_shared<std::thread>(
            std::bind(&TurtleSpawnerNode::callSpawnService, this, name, x, y, theta)));
    }

    void callSpawnService(const std::string turtle_name, const double x, const double y, const double theta)
    {
        while(! this->spawn_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for spawn service...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        request->name = turtle_name;

        auto future = spawn_client_->async_send_request(request);
        
        try
        {
            auto response = future.get();

            if(response->name != "")
            {
                
                auto new_turtle = turtle_interfaces::msg::Turtle();
                new_turtle.name = response->name;
                new_turtle.x = request->x;
                new_turtle.y = request->y;
                new_turtle.theta = request->theta;    
                alive_turtles_.push_back(new_turtle);
                publishAliveTurtles();            
                
                RCLCPP_INFO(get_logger(),"New alive turtle: %s", response->name.c_str());

            }
            
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Spawn Service call failed");
        }
        
    }

    void publishAliveTurtles()
    {
        auto msg = turtle_interfaces::msg::TurtleArray();
        msg.turtles = alive_turtles_;
        alive_turtle_publisher_->publish(msg);
    }

    void callbackCatchTurtle(const turtle_interfaces::srv::CatchTurtle::Request::SharedPtr request,
                             const turtle_interfaces::srv::CatchTurtle::Response::SharedPtr response)
    {
        kill_turtle_threads_.push_back(std::make_shared<std::thread>(std::bind(&TurtleSpawnerNode::callKillTurtleService, this, request->name)));
        response->success = true;
    }

    void callKillTurtleService(std::string turtle_name)
    {
        while(! kill_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for kill server...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turtle_name;

        auto future = kill_client_->async_send_request(request);

        try
        {
            auto response = future.get();
            
            for(int i = 0; (int)alive_turtles_.size(); i++)
            {
                if(alive_turtles_.at(i).name == turtle_name)
                {
                    alive_turtles_.erase(alive_turtles_.begin() + i);
                    publishAliveTurtles();
                    break;
                }
            }

        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Kill service call failed.");
        }

    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}