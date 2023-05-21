#include "d1_node.h"

int main(int argc, char **argv)
{
    using namespace D1;
    ros::init(argc, argv, "cyglidar_publisher");
    ros::NodeHandle nh;

    CygLiDAR_D1 node(nh);

    ros::spin();
    return 0;
}
