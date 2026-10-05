#include "ur_pingpong_cmd_publisher/cmd_publisher_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdPublisherNode>();
    auto executor = rclcpp::executors::MultiThreadedExecutor();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}
