#include "D1_Node.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<D1_Node> d1_node = std::make_shared<D1_Node>();

    try
    {
        d1_node->connectBoostSerial();

        while(rclcpp::ok())
        {
            d1_node->loopCygParser();
        }

        d1_node->disconnectBoostSerial();
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[D1 NODE ERROR] : %s", e.what());
    }

    rclcpp::shutdown();
}
