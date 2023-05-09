#include "serial.h"
#include "point_cloud_maker.h"
#include "d_series_constant.h"
#include "cyglidar_driver.h"
#include "point_cloud_maker.h"
#include "d1_2d_topic.h"
#include "d1_3d_topic.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
    std::string port;
    int baud_rate;
    std::string frame_id;
    int run_mode;
    int duration_mode;
    int duration_value;
    int frequency_channel;

    ros::init(argc, argv, "cyglidar_publisher");

    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    priv_nh.param<std::string>("port", port, "/dev/ttyUSB0");
    priv_nh.param<int>("baud_rate", baud_rate, 3000000);
    priv_nh.param<std::string>("frame_id", frame_id, "laser_frame");
    priv_nh.param<int>("run_mode", run_mode, 2);
    priv_nh.param<int>("duration_mode", duration_mode, PULSE_AUTO);
    priv_nh.param<int>("duration_value", duration_value, 10000);

    ColorRGB colorRGB;
    D1::Topic_2D  Topic_2D;
    D1::Topic_3D  Topic_3D;
    cyg_driver::TransformPayload TransformPayload;

    // Call the following function so as to store colors to draw 3D data
    colorRGB.initColorMap();

    float param_x[CygLiDARD1::Sensor::numPixel], param_y[CygLiDARD1::Sensor::numPixel], param_z[CygLiDARD1::Sensor::numPixel];
    PointCloudMaker pointcloud_3d = PointCloudMaker(param_x, param_y, param_z, CygLiDARD1::Sensor::numPixel);

    pointcloud_3d.initLensTransform(CygLiDARD1::Sensor::PixelRealSize, CygLiDARD1::Sensor::Width, CygLiDARD1::Sensor::Height,
                                    CygLiDARD1::Parameter::OffsetCenterPoint_x, CygLiDARD1::Parameter::OffsetCenterPoint_y);

    boost::asio::io_service io;

    try
    {
        // Open the port
        cyglidar_pcl serial_port(port, baud_rate, io);

        ros::Publisher publisher_image      = nh.advertise<sensor_msgs::Image>      ("scan_image", 5);
        ros::Publisher publisher_laserscan  = nh.advertise<sensor_msgs::LaserScan>  ("scan", 1000);
        ros::Publisher publisher_point_2d   = nh.advertise<sensor_msgs::PointCloud2>("scan_2D", 5);
        ros::Publisher publisher_point_3d   = nh.advertise<sensor_msgs::PointCloud2>("scan_3D", 5);

        ROS_INFO("%s", serial_port.requestRunMode(static_cast<eRunMode>(run_mode)).c_str());

        serial_port.requestDurationControl(static_cast<eRunMode>(run_mode), duration_mode, duration_value);
        ros::Duration(1.0).sleep();
        ROS_INFO("[PACKET UPDATED] PULSE DURATION : %d", duration_value);
        // sleep for a sec, by the duration

        serial_port.requestFrequencyChannel(frequency_channel);
        ROS_INFO("[PACKET UPDATED] FREQUENCY CH.%d", frequency_channel);

        sensor_msgs::Image::Ptr                 scan_image(new sensor_msgs::Image);
	    sensor_msgs::LaserScan::Ptr             scan_laser(new sensor_msgs::LaserScan);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_2D(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_3D(new pcl::PointCloud<pcl::PointXYZRGBA>);

        ros::Time scan_start_time = ros::Time::now() - ros::Duration(0.00048);
        double scan_duration = 0;

        uint8_t total_packet_data[SCAN_MAX_SIZE];
        uint8_t packet_structure[SCAN_MAX_SIZE];

        uint16_t distance_buffer_2d[cyg_driver::DATA_LENGTH_2D];
        uint16_t distance_buffer_3d[cyg_driver::DATA_LENGTH_3D];
        uint16_t number_of_data = 0;

        while (ros::ok())
        {
            number_of_data = serial_port.getPacketLength(packet_structure, SCAN_MAX_SIZE);
            for (int i = 0; i < number_of_data; i++)
            {
                if(CygParser(total_packet_data, packet_structure[i]) == CHECKSUM_PASSED)
                {
                    if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_2D)
                    {
                        TransformPayload.getDistanceArray2D(&total_packet_data[PAYLOAD_DATA], distance_buffer_2d);

                        Topic_2D.publishScanLaser(publisher_laserscan, scan_laser, frame_id, scan_start_time, scan_duration, distance_buffer_2d);
                        Topic_2D.publishPoint2D(publisher_point_2d, scan_2D, frame_id, distance_buffer_2d);
                    }
                    else if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_3D)
                    {
                        TransformPayload.getDistanceArray3D(&total_packet_data[PAYLOAD_DATA], distance_buffer_3d);

                        Topic_3D.publishScanImage(publisher_image, scan_image, frame_id, distance_buffer_3d);
                        Topic_3D.publishPoint3D(publisher_point_3d, scan_3D, frame_id, pointcloud_3d, colorRGB.color_map, distance_buffer_3d);
                    }
                }
            }
        }
        serial_port.close();
        ROS_INFO("PACKET UPDATED : STOP");
        return 0;
    }
    catch (const boost::system::system_error& ex)
    {
        ROS_ERROR("An exception was thrown [%s]", ex.what());
        return -1;
    }
}
