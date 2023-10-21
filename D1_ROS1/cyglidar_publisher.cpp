#include "d1_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "D1_NODE");

    std::shared_ptr<D1_Node> d1_node = std::make_shared<D1_Node>();

    try
    {
        d1_node->connectBoostSerial();

        while(ros::ok())
        {
            d1_node->loopCygParser();
        }

        d1_node->disconnectBoostSerial();
    }
    catch (const ros::Exception &e)
    {
        ROS_ERROR("[D1 NODE ERROR] : %s", e.what());
    }

    ros::shutdown();
}
