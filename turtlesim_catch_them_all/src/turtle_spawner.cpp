#include "rclcpp/rclcpp.hpp"

class TurtleSpawnerNode : public rclcpp::Node
{
    public:
        TurtleSpawnerNode() : Node("turtle_spawner")
        {

        }

    private:

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}