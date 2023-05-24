#include "d1_node.h"

namespace D1
{
    using namespace CygLiDARD1;

    CygLiDAR_D1::CygLiDAR_D1(ros::NodeHandle nh_) : nh(nh_)
    {
        publisher_laserscan  = nh.advertise<sensor_msgs::LaserScan>  ("scan",    1000);
        publisher_point_2d   = nh.advertise<sensor_msgs::PointCloud2>("scan_2D",    5);
        publisher_point_3d   = nh.advertise<sensor_msgs::PointCloud2>("scan_3D",    5);
        publisher_image      = nh.advertise<sensor_msgs::Image>      ("scan_image", 5);
        
        initParam();
        onInit();
    }

    float param_x[Sensor::numPixel], param_y[Sensor::numPixel], param_z[Sensor::numPixel];
    PointCloudMaker pointcloud_maker = PointCloudMaker(param_x, param_y, param_z, Sensor::numPixel);
    
    void CygLiDAR_D1::initParam()
    {
        ros::NodeHandle priv_nh("~");
        priv_nh.param<std::string>("port", port, "/dev/ttyUSB0");
        priv_nh.param<int>("baud_rate", baud_rate, 3000000);
        priv_nh.param<std::string>("frame_id", frame_id, "laser_frame");
        priv_nh.param<int>("run_mode", run_mode, 2);
        priv_nh.param<int>("duration_mode", duration_mode, PULSE_AUTO);
        priv_nh.param<int>("duration_value", duration_value, 10000);          
        
        // Call the following function so as to store colors to draw 3D data
        pointcloud_maker.initColorMap();
        pointcloud_maker.initLensTransform(Sensor::PixelRealSize, Sensor::Width, Sensor::Height,
                                           Parameter::OffsetCenterPoint_x, Parameter::OffsetCenterPoint_y);
    }

    void CygLiDAR_D1::onInit()
    {
        boost::asio::io_service io;

        try
        {
            // Open the port
            cyglidar_serial serial_port(port, baud_rate, io);

            sendPacketData(serial_port);
            publishLoop(serial_port);
        }
        catch (const boost::system::system_error& ex)
        {
            ROS_ERROR("An exception was thrown [%s]", ex.what());
        }
    }

    void CygLiDAR_D1::publishLoop(cyglidar_serial &serial_port_)
    {
        while (ros::ok())
        {
            number_of_data = serial_port_.getPacketLength(packet_structure, SCAN_MAX_SIZE);

            for (int i = 0; i < number_of_data; i++)
            {
                if(CygParser(total_packet_data, packet_structure[i]) == CHECKSUM_PASSED)
                {
                    publishData(total_packet_data);
                }
            }
        }
        serial_port_.close();
        ROS_INFO("PACKET REQUEST : STOP");
    }

    void CygLiDAR_D1::publishData(uint8_t *total_packet_data_)
    {
        Topic_2D Topic_data_2d;
        Topic_3D Topic_data_3d;
        cyg_driver::TransformPayload TransformPayload;

        ros::Time scan_start_time = ros::Time::now() - ros::Duration(0.00048);

        if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_2D)
        {
            TransformPayload.getDistanceArray2D(&total_packet_data[PAYLOAD_DATA], distance_buffer_2d);

            Topic_data_2d.publishScanLaser(frame_id, publisher_laserscan, scan_start_time, distance_buffer_2d);
            Topic_data_2d.publishPoint2D(frame_id, publisher_point_2d, distance_buffer_2d);
        }
        else if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_3D)
        {
            TransformPayload.getDistanceArray3D(&total_packet_data[PAYLOAD_DATA], distance_buffer_3d);

            Topic_data_3d.publishScanImage(frame_id, publisher_image, distance_buffer_3d);
            Topic_data_3d.publishPoint3D(frame_id, publisher_point_3d, pointcloud_maker, distance_buffer_3d);
        }
        else if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_DEV_INFO)
        {
            ROS_INFO("[F/W VERSION] %d.%d.%d", total_packet_data[6], total_packet_data[7], total_packet_data[8]);
            ROS_INFO("[H/W VERSION] %d.%d.%d", total_packet_data[9], total_packet_data[10], total_packet_data[11]);
        }
    }

    void CygLiDAR_D1::sendPacketData(cyglidar_serial &serial_port_)
    {
        serial_port_.requestDeviceInfo();
        ros::Duration(3.0).sleep();
        // sleep for 3s, by requsting the info data.

        ROS_INFO("%s", serial_port_.requestRunMode(static_cast<eRunMode>(run_mode)).c_str());

        serial_port_.requestDurationControl(static_cast<eRunMode>(run_mode), duration_mode, duration_value);
        ros::Duration(1.0).sleep();
        ROS_INFO("[PACKET REQUEST] PULSE DURATION : %d", duration_value);
        // sleep for a sec, by the duration

        serial_port_.requestFrequencyChannel(frequency_channel);
        ROS_INFO("[PACKET REQUEST] FREQUENCY CH.%d", frequency_channel);
    }
}
