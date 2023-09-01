#include "d1_node.h"

using namespace std::chrono_literals;
using namespace Constant_D1;

D1_Node::D1_Node(rclcpp::Node::SharedPtr _node) : node(_node)
{
    topic_2d = new Topic2D();
    topic_3d = new Topic3D();

    topic_2d->initPublisher(node->create_publisher<sensor_msgs::msg::LaserScan>  ("scan",    10),
                            node->create_publisher<sensor_msgs::msg::PointCloud2>("scan_2D", 10));

    topic_3d->initPublisher(node->create_publisher<sensor_msgs::msg::Image>      ("scan_image", 10),
                            node->create_publisher<sensor_msgs::msg::PointCloud2>("scan_3D",    10));

    received_buffer[0].packet_data = first_total_packet_data;
    received_buffer[1].packet_data = second_total_packet_data;

    initConfiguration();

    future = exit_signal.get_future();
    double_buffer_thread = std::thread(&D1_Node::doublebufferThread, this);
    publish_thread = std::thread(&D1_Node::publishThread, this);
}

D1_Node::~D1_Node()
{
    exit_signal.set_value();
    double_buffer_thread.join();
    publish_thread.join();
    delete topic_2d;
    delete topic_3d;
    delete serial_port;
}

void D1_Node::publishThread()
{
    do
    {
        runPublish();
        status = future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
}

void D1_Node::runPublish()
{
    if (publish_data_state == PUBLISH_3D)
    {
        publish_data_state == PUBLISH_DONE;
        topic_3d->publishScanImage(distance_buffer_3d);
        topic_3d->publishPoint3D(distance_buffer_3d);
    }
    else if (publish_data_state == PUBLISH_2D)
    {
        publish_data_state == PUBLISH_DONE;
        topic_2d->publishScanLaser(scan_start_time, distance_buffer_2d);
        topic_2d->publishPoint2D(distance_buffer_2d);
    }
}

void D1_Node::doublebufferThread()
{
    do
    {
        processDoubleBuffer();
        status = future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
}

void D1_Node::processDoubleBuffer()
{
    if(publish_done_flag & 0x1)
    {
        publish_done_flag &= (~0x1);
        convertData(&received_buffer[0]);
    }
    else if(publish_done_flag & 0x2)
    {
        publish_done_flag &= (~0x2);
        convertData(&received_buffer[1]);
    }
}

void D1_Node::initConfiguration()
{
    port              = node->declare_parameter("port", "/dev/ttyUSB0");
    baud_rate         = node->declare_parameter("baud_rate", 3000000);
    frame_id          = node->declare_parameter("frame_id", "laser_frame");
    run_mode          = node->declare_parameter("run_mode", 2);
    duration_mode     = node->declare_parameter("duration_mode", PULSE_AUTO);
    duration_value    = node->declare_parameter("duration_value", 10000);
    frequency_channel = node->declare_parameter("frequency_channel", 0);

    topic_2d->assignLaserScan(frame_id);
    topic_2d->assignPCL2D(frame_id);
    topic_3d->assignImage(frame_id);
    topic_3d->assignPCL3D(frame_id);
}

void D1_Node::connectBoostSerial()
{
    try
    {
        // Open the port
        serial_port = new cyglidar_serial(port, baud_rate, io_service);

        requestPacketData();
    }
    catch (const boost::system::system_error& ex)
    {
        RCLCPP_ERROR(node->get_logger(), "[BOOST SERIAL ERROR] %s", ex.what());
    }
}

void D1_Node::disconnectBoostSerial()
{
    serial_port->close();
    RCLCPP_INFO(node->get_logger(), "[PACKET REQUEST] STOP");
}

void D1_Node::loopCygParser()
{
    number_of_data = serial_port->getPacketLength(packet_structure, SCAN_MAX_SIZE);

    for (uint16_t i = 0; i < number_of_data; i++)
    {
        parser_return = CygParser(received_buffer[double_buffer_index].packet_data, packet_structure[i]);

        if(parser_return == CHECKSUM_PASSED)
        {
            publish_done_flag |= (1 << double_buffer_index);

            double_buffer_index++;
            double_buffer_index &= 1;

            if (received_buffer->packet_data[PAYLOAD_HEADER] == PACKET_HEADER_DEV_INFO && info_flag == false)
            {
                RCLCPP_INFO(node->get_logger(), "[F/W VERSION] %d.%d.%d", received_buffer->packet_data[6], received_buffer->packet_data[7], received_buffer->packet_data[8]);
                RCLCPP_INFO(node->get_logger(), "[H/W VERSION] %d.%d.%d", received_buffer->packet_data[9], received_buffer->packet_data[10], received_buffer->packet_data[11]);
                info_flag = true;
            }
        }
    }
}

void D1_Node::convertData(received_data_buffer* _received_buffer)
{
    if (_received_buffer->packet_data[PAYLOAD_HEADER] == PACKET_HEADER_2D)
    {
        scan_start_time = node->now() - rclcpp::Duration(0, 48*1000*1000);

        TransformPayload.getDistanceArray2D(&_received_buffer->packet_data[PAYLOAD_DATA], distance_buffer_2d);
        publish_data_state = PUBLISH_2D;
    }
    else if (_received_buffer->packet_data[PAYLOAD_HEADER] == PACKET_HEADER_3D)
    {
        TransformPayload.getDistanceArray3D(&_received_buffer->packet_data[PAYLOAD_DATA], distance_buffer_3d);
        publish_data_state = PUBLISH_3D;
    }    
}

void D1_Node::requestPacketData()
{
    serial_port->requestDeviceInfo();
    std::this_thread::sleep_for(3s);
    // sleep for 3s, by requsting the info data.

    serial_port->requestRunMode(static_cast<eRunMode>(run_mode), mode_notice);
    RCLCPP_INFO(node->get_logger(), "[PACKET REQUEST] %s", mode_notice.c_str());

    serial_port->requestDurationControl(static_cast<eRunMode>(run_mode), duration_mode, duration_value);
    std::this_thread::sleep_for(1s);
    RCLCPP_INFO(node->get_logger(), "[PACKET REQUEST] PULSE DURATION : %d", duration_value);
    // sleep for a sec, by the duration

    serial_port->requestFrequencyChannel(frequency_channel);
    RCLCPP_INFO(node->get_logger(), "[PACKET REQUEST] FREQUENCY CH.%d", frequency_channel);
}

