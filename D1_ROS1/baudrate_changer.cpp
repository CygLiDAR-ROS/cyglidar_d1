#include <ros/ros.h>
#include "serial.h"

class BaudrateChangerNode
{
    public:
        BaudrateChangerNode(ros::NodeHandle _nh) : nh(_nh)
        {
            ros::NodeHandle priv_nh("~");
            priv_nh.param<std::string>("port", port, "/dev/ttyUSB0");
            priv_nh.param<int>("current_baud_rate", baud_rate, 3000000);
            priv_nh.param<int>("select_baud_rate", select_baud_rate, 2);
        }

        ~BaudrateChangerNode() { delete serial_port; }

        void connectBoostSerial()
        {
            try
            {
                // Open the port
                serial_port = new cyglidar_serial(port, baud_rate, io);
            }
            catch (const boost::system::system_error& ex)
            {
                ROS_ERROR("[BOOST SERIAL ERROR] : %s", ex.what());
            }
        }

        void requestPacketData()
        {
            if (select_baud_rate == BAUDRATE_3000000)
            {
                BAUDRATE_COMMAND = 0x55;
                ROS_INFO("[PACKET REQUEST] CHANGE BAUDRATE -> 3,000,000");
            }
            else if (select_baud_rate == BAUDRATE_115200)
            {
                BAUDRATE_COMMAND = 0xAA;
                ROS_INFO("[PACKET REQUEST] CHANGE BAUDRATE -> 115,200");
            }
            else if (select_baud_rate == BAUDRATE_57600)
            {
                BAUDRATE_COMMAND = 0x39;
                ROS_INFO("[PACKET REQUEST] CHANGE BAUDRATE -> 57,600");
            }
            serial_port->requestSerialBaudRate(BAUDRATE_COMMAND);
        }

    private:
        cyglidar_serial *serial_port;

        boost::asio::io_service io;

        ros::NodeHandle nh;

        std::string port;
        int baud_rate;
        int select_baud_rate;

        const uint8_t BAUDRATE_3000000 = 0;
        const uint8_t BAUDRATE_115200  = 1;
        const uint8_t BAUDRATE_57600   = 2;

        uint8_t BAUDRATE_COMMAND;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "baudrate_changer");
    ros::NodeHandle nh;

    std::shared_ptr<BaudrateChangerNode> baud_rate_changer_node = std::make_shared<BaudrateChangerNode>(nh);

    try
    {
        baud_rate_changer_node->connectBoostSerial();

        baud_rate_changer_node->requestPacketData();
    }
    catch (const ros::Exception &e)
    {
        ROS_ERROR("[BAUDRATE CHANGER ERROR] : %s", e.what());
    }

    ros::shutdown();
}
