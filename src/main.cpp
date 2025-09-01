#include <rclcpp/rclcpp.hpp>
#include "flocking_algorithm_ros2/flocking_node.hpp"


int main(int argc, char ** argv)
{
rclcpp::init(argc, argv);
auto node = std::make_shared<flocking::FlockingNode>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}