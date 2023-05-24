#include "d1_node.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<rclcpp::Node> node = std::make_shared<D1::CygLiDAR_D1>("cyglidar_publisher");

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}
