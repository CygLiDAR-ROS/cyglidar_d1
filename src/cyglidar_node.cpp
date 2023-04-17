#include "serial.h"
#include "point_cloud_maker.h"
#include "d_series_constant.h"
#include "cyglidar_driver.h"

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

ColorRGB colorRGB;

void publishMessageLaserScan(ros::Publisher publisher_laserscan_, sensor_msgs::LaserScan::Ptr message_laserscan_, std::string frame_id_,
                             ros::Time start_, double scan_time_, uint16_t *Distance2D_)
{
    message_laserscan_->header.frame_id = frame_id_;
    message_laserscan_->header.stamp = start_;
    message_laserscan_->angle_min = -static_cast<double>(CygLiDARD1::Sensor::HorizontalAngle / 2.0f * CygLiDARD1::Util::ToRadian);
    message_laserscan_->angle_max = static_cast<double>(CygLiDARD1::Sensor::HorizontalAngle / 2.0f * CygLiDARD1::Util::ToRadian);
    message_laserscan_->angle_increment = static_cast<double>(CygLiDARD1::Sensor::AngleIncremet2D * CygLiDARD1::Util::ToRadian);
    message_laserscan_->scan_time = scan_time_;
    message_laserscan_->range_min = static_cast<double>(CygLiDARD1::Distance::Mode2D::Minimum_Depth_2D * CygLiDARD1::Util::MM_To_M);
    message_laserscan_->range_max = static_cast<double>(CygLiDARD1::Distance::Mode2D::Maximum_Depth_2D * CygLiDARD1::Util::MM_To_M);
    message_laserscan_->ranges.resize(cyg_driver::DATA_LENGTH_2D);
    message_laserscan_->intensities.resize(cyg_driver::DATA_LENGTH_2D);

    for (int i = 0; i < cyg_driver::DATA_LENGTH_2D; i++)
    {
        int data_index = (cyg_driver::DATA_LENGTH_2D - 1 - i);
        if (Distance2D_[data_index] < (float)(CygLiDARD1::Distance::Mode2D::Maximum_Depth_2D))
        {
            message_laserscan_->ranges[i] = Distance2D_[data_index] * MM2M;
        }
        else
        {
            message_laserscan_->ranges[i] = std::numeric_limits<float>::infinity();
        }
    }
    publisher_laserscan_.publish(message_laserscan_);
}

void publishMessagePoint2D(ros::Publisher publisher_point_2d_, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr message_point_2d_,
                           std::string frame_id_, uint16_t *Distance2D_)
{
    message_point_2d_->header.frame_id = frame_id_;
    message_point_2d_->is_dense = false;
    message_point_2d_->points.resize(cyg_driver::DATA_LENGTH_2D);

    double angle_increment_steps = static_cast<double>(CygLiDARD1::Sensor::AngleIncremet2D);

    int data_idx;
    uint16_t raw_distance;
    float point_angle_2d, point_angle_variable_2d;
    float world_coordinate_x, world_coordinate_y;

    for (int i = 0; i < cyg_driver::DATA_LENGTH_2D; i++)
    {
        // Reverse data order of the array
        data_idx = (cyg_driver::DATA_LENGTH_2D - 1 - i);

        raw_distance = (distance_buffer_2d_[data_idx]);

        point_angle_2d = (float)(((-HORIZONTAL_ANGLE / 2)+ point_angle_variable_2d) * CygLiDARD1::Util::ToRadian);
        point_angle_variable_2d += angle_increment_steps;

        world_coordinate_x = (sin(point_angle_2d) * raw_distance);
        world_coordinate_y = (cos(point_angle_2d) * raw_distance);

        pointcloud_2d_->points[i].x = world_coordinate_y * MM2M;
        pointcloud_2d_->points[i].y = world_coordinate_x * MM2M;
        pointcloud_2d_->points[i].z = 0.0;

        if (distance_buffer_2d_[data_idx] < CygLiDARD1::Distance::Mode2D::Maximum_Depth_2D)
        {
            pointcloud_2d_->points[i].r = 255;
            pointcloud_2d_->points[i].g = 255;
            pointcloud_2d_->points[i].b = 0;
            pointcloud_2d_->points[i].a = 255;
        }
        else
        {
            // Turn data invisible when it's greater than the maximum
            pointcloud_2d_->points[i].a = 0;
        }
    }
    pcl_conversions::toPCL(ros::Time::now(), message_point_2d_->header.stamp);
    publisher_point_2d_.publish(message_point_2d_);
}

void publishMessagePoint3D(ros::Publisher publisher_point_3d_, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr message_point_3d_,
                           std::string frame_id_, PointCloudMaker &PointCloud, uint16_t *Distance3D_)
{
    message_point_3d_->header.frame_id = frame_id_;
    message_point_3d_->is_dense = false;
    message_point_3d_->width  = CygLiDARD1::Sensor::Width;
    message_point_3d_->height = CygLiDARD1::Sensor::Height;
    message_point_3d_->points.resize(cyg_driver::DATA_LENGTH_3D);

    uint16_t raw_distance;
    float world_coordinate_x, world_coordinate_y, world_coordinate_z;
	
    for (int buffer_index = 0; buffer_index < CygLiDARD1::Sensor::Height * CygLiDARD1::Sensor::Width; buffer_index++)
    {
        raw_distance = distance_buffer_3d_[buffer_index];

        if(raw_distance < CygLiDARD1::Distance::Mode3D::Maximum_Depth_3D)
        {
            PointCloud.calcPointCloud(raw_distance, buffer_index, world_coordinate_x, world_coordinate_y, world_coordinate_z);

            pointcloud_3d_->points[buffer_index].x = world_coordinate_z * MM2M;
            pointcloud_3d_->points[buffer_index].y = -world_coordinate_x * MM2M;
            pointcloud_3d_->points[buffer_index].z = -world_coordinate_y * MM2M;
            uint32_t color_change_with_height = colorRGB.color_map[((int)world_coordinate_y / 2) % colorRGB.color_map.size()];
            pointcloud_3d_->points[buffer_index].rgb = *reinterpret_cast<float *>(&color_change_with_height);
            pointcloud_3d_->points[buffer_index].a = 255;
        }
        else
        {
            pointcloud_3d_->points[buffer_index].x = 0;
            pointcloud_3d_->points[buffer_index].y = 0;
            pointcloud_3d_->points[buffer_index].z = 0;
            pointcloud_3d_->points[buffer_index].rgb = 0;
            pointcloud_3d_->points[buffer_index].a = 0;
        }
    }
    pcl_conversions::toPCL(ros::Time::now(), message_point_3d_->header.stamp);
    publisher_point_3d_.publish(message_point_3d_);
}

int main(int argc, char **argv)
{
    std::string port;
    int baud_rate;
    std::string frame_id;
    int run_mode;
    int duration_mode;
    int duration_value;
    int frequency_channel;
    int sensitivity;

    ros::init(argc, argv, "Cyglidar_Node");

    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    priv_nh.param<std::string>("port", port, "/dev/ttyUSB0");
    priv_nh.param<int>("baud_rate", baud_rate, 3000000);
    priv_nh.param<std::string>("frame_id", frame_id, "laser_frame");
    priv_nh.param<int>("run_mode", run_mode, 2);
    priv_nh.param<int>("duration_mode", duration_mode, PULSE_AUTO);
    priv_nh.param<int>("duration_value", duration_value, 10000);
    priv_nh.param<int>("frequency_channel", frequency_channel, 0);

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

        ros::Publisher publisher_laserscan  = nh.advertise<sensor_msgs::LaserScan>("scan", SCAN_MAX_SIZE);
        ros::Publisher publisher_point_2d   = nh.advertise<sensor_msgs::PointCloud2>("scan_2D", 1);
        ros::Publisher publisher_point_3d   = nh.advertise<sensor_msgs::PointCloud2>("scan_3D", 1);

        ROS_INFO("%s", serial_port.requestRunMode(static_cast<eRunMode>(run_mode)).c_str());

        
        serial_port.requestDurationControl(static_cast<eRunMode>(run_mode), duration_mode, duration_value);
        ros::Duration(1.0).sleep(); // sleep for a sec, by the duration        
	ROS_INFO("[PACKET UPDATED] PULSE DURATION : %d", duration_value);
        
        serial_port.requestFrequencyChannel(frequency_channel);
	ROS_INFO("[PACKET UPDATED] FREQUENCY CH.%d", frequency_channel);

	sensor_msgs::LaserScan::Ptr             scan_laser(new sensor_msgs::LaserScan);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_2D(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_3D(new pcl::PointCloud<pcl::PointXYZRGBA>);

        ros::Time scan_start_time = ros::Time::now() - ros::Duration(0.00015); //nanosec
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

                        publishMessageLaserScan(publisher_laserscan, scan_laser, frame_id, scan_start_time, scan_duration, distance_buffer_2d);
                        publishMessagePoint2D(publisher_point_2d, scan_2D, frame_id, distance_buffer_2d);
                    }
                    else if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_3D)
                    {
                        TransformPayload.getDistanceArray3D(&total_packet_data[PAYLOAD_DATA], distance_buffer_3d);

                        publishMessagePoint3D(publisher_point_3d, scan_3D, frame_id, pointcloud_3d, distance_buffer_3d);
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
