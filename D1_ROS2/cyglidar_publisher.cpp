#include "d1_node.h"

D1_Node *D1_node;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("D1_NODE");

    D1_node = new D1_Node(node);

    try
    {    
        D1_node->connectBoostSerial();

        while(rclcpp::ok())
        {
            D1_node->loopCygParser();
        }

        D1_node->disconnectBoostSerial();
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
        RCLCPP_ERROR(node->get_logger(), "[D1 NODE ERROR] : %s", e.what());
    }

    rclcpp::shutdown();
    delete D1_node;
}
