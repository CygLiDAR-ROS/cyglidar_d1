#include "d1_node.h"

D1_Node *D1_node;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "D1_NODE");
    ros::NodeHandle nh;

    D1_node = new D1_Node(nh);
    try
    {
        D1_node->connectBoostSerial();
        
        while(ros::ok())
        {
            D1_node->loopCygParser();
        }

        D1_node->disconnectBoostSerial();
    }
    catch (const ros::Exception &e)
    {
        ROS_ERROR("[D1 NODE ERROR] : %s", e.what());
    }

    ros::shutdown();
    delete D1_node;
}
