#include <rclcpp/rclcpp.hpp>
#include "serial.h"

class BaudrateChangerNode
{
    public:
        BaudrateChangerNode(rclcpp::Node::SharedPtr _node) : node(_node)
        {
            node->declare_parameter("port");
            node->declare_parameter("current_baud_rate");
            node->declare_parameter("select_baud_rate");

            node->get_parameter_or<std::string>("port", port, "/dev/ttyS13");
            node->get_parameter_or<int>("current_baud_rate", baud_rate, 3000000);
            node->get_parameter_or<uint8_t>("select_baud_rate", select_baud_rate, 2);
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
                RCLCPP_ERROR(node->get_logger(), "[BOOST SERIAL ERROR] : %s", ex.what());
            }
        }

        void requestPacketData()
        {
            if (select_baud_rate == BAUDRATE_3000000)
            {
                BAUDRATE_COMMAND = 0x55;
                RCLCPP_INFO(node->get_logger(), "[PACKET REQUEST] CHANGE BAUDRATE -> 3,000,000");
            }
            else if (select_baud_rate == BAUDRATE_115200)
            {
                BAUDRATE_COMMAND = 0xAA;
                RCLCPP_INFO(node->get_logger(), "[PACKET REQUEST] CHANGE BAUDRATE -> 115,200");
            }
            else if (select_baud_rate == BAUDRATE_57600)
            {
                BAUDRATE_COMMAND = 0x39;
                RCLCPP_INFO(node->get_logger(), "[PACKET REQUEST] CHANGE BAUDRATE -> 57,600");
            }
            serial_port->requestSerialBaudRate(BAUDRATE_COMMAND);
        }

    private:
        cyglidar_serial *serial_port;

        boost::asio::io_service io;

        rclcpp::Node::SharedPtr node;

        std::string port;
        int baud_rate;
        uint8_t select_baud_rate;

        const uint8_t BAUDRATE_3000000 = 0;
        const uint8_t BAUDRATE_115200  = 1;
        const uint8_t BAUDRATE_57600   = 2;

        uint8_t BAUDRATE_COMMAND;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("baudrate_changer");
    std::shared_ptr<BaudrateChangerNode> baud_rate_changer_node = std::make_shared<BaudrateChangerNode>(node);

    try
    {    
        baud_rate_changer_node->connectBoostSerial();

        baud_rate_changer_node->requestPacketData();
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
        RCLCPP_ERROR(node->get_logger(), "[BAUDRATE CHANGER ERROR] : %s", e.what());
    }

    rclcpp::shutdown();
}
