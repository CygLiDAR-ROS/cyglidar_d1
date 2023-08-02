#include "d1_node.h"

using namespace Constant_D1;

D1_Node::D1_Node(ros::NodeHandle _nh) : nh(_nh)
{
    topic_2d = new Topic2D();
    topic_3d = new Topic3D();

    topic_2d->initPublisher(nh.advertise<sensor_msgs::LaserScan>  ("scan",    10),
                            nh.advertise<sensor_msgs::PointCloud2>("scan_2D", 10));

    topic_3d->initPublisher(nh.advertise<sensor_msgs::Image>      ("scan_image", 10),
                            nh.advertise<sensor_msgs::PointCloud2>("scan_3D",    10));

    received_buffer[0].packet_data = first_total_packet_data;
    received_buffer[1].packet_data = second_total_packet_data;

    initConfiguration();
    
    future = exit_signal.get_future();
    publish_thread = std::thread(&D1_Node::publishThread, this);
}

D1_Node::~D1_Node()
{
    exit_signal.set_value();
    publish_thread.join();
    delete topic_2d;
    delete topic_3d;
    delete serial_port;
}

void D1_Node::publishThread()
{
    std::future_status status;

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
    ros::NodeHandle priv_nh("~");
    priv_nh.param<std::string>("port", port, "/dev/ttyUSB0");
    priv_nh.param<int>("baud_rate", baud_rate, 3000000);
    priv_nh.param<std::string>("frame_id", frame_id, "laser_frame");
    priv_nh.param<int>("run_mode", run_mode, 2);
    priv_nh.param<int>("duration_mode", duration_mode, PULSE_AUTO);
    priv_nh.param<int>("duration_value", duration_value, 10000);
    priv_nh.param<int>("frequency_channel", frequency_channel, 0);

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
        serial_port = new cyglidar_serial(port, baud_rate, io);

        requestPacketData();
    }
    catch (const boost::system::system_error& ex)
    {
        ROS_ERROR("[BOOST SERIAL ERROR] %s", ex.what());
    }
}

void D1_Node::disconnectBoostSerial()
{
    serial_port->close();
    ROS_ERROR("[PACKET REQUEST] STOP");
}

void D1_Node::loopCygParser()
{
    number_of_data = serial_port->getPacketLength(packet_structure, SCAN_MAX_SIZE);

    for (int i = 0; i < number_of_data; i++)
    {
        parser_return = CygParser(received_buffer[double_buffer_index].packet_data, packet_structure[i]);

        if(parser_return == CHECKSUM_PASSED)
        {
            publish_done_flag |= (1 << double_buffer_index);

            double_buffer_index++;
            double_buffer_index &= 1;

            processDoubleBuffer();
        }
    }
}

void D1_Node::convertData(received_data_buffer *_received_buffer)
{
    cyg_driver::TransformPayload TransformPayload;

    if (_received_buffer->packet_data[PAYLOAD_HEADER] == PACKET_HEADER_2D)
    {
        scan_start_time = ros::Time::now() - ros::Duration(0.00048);

        TransformPayload.getDistanceArray2D(&_received_buffer->packet_data[PAYLOAD_DATA], distance_buffer_2d);
        publish_data_state = PUBLISH_2D;
    }
    else if (_received_buffer->packet_data[PAYLOAD_HEADER] == PACKET_HEADER_3D)
    {
        TransformPayload.getDistanceArray3D(&_received_buffer->packet_data[PAYLOAD_DATA], distance_buffer_3d);
        publish_data_state = PUBLISH_3D;
    }
    else if (_received_buffer->packet_data[PAYLOAD_HEADER] == PACKET_HEADER_DEV_INFO)
    {
        ROS_INFO("[F/W VERSION] %d.%d.%d", _received_buffer->packet_data[6], _received_buffer->packet_data[7], _received_buffer->packet_data[8]);
        ROS_INFO("[H/W VERSION] %d.%d.%d", _received_buffer->packet_data[9], _received_buffer->packet_data[10], _received_buffer->packet_data[11]);
    }
}

void D1_Node::requestPacketData()
{
    serial_port->requestDeviceInfo();
    ros::Duration(3.0).sleep();
    // sleep for 3s, by requsting the info data.

    ROS_INFO("%s", serial_port->requestRunMode(static_cast<eRunMode>(run_mode)));

    serial_port->requestDurationControl(static_cast<eRunMode>(run_mode), duration_mode, duration_value);
    ros::Duration(1.0).sleep();
    ROS_INFO("[PACKET REQUEST] PULSE DURATION : %d", duration_value);
    // sleep for a sec, by the duration

    serial_port->requestFrequencyChannel(frequency_channel);
    ROS_INFO("[PACKET REQUEST] FREQUENCY CH.%d", frequency_channel);
}
