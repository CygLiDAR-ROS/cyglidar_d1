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
    message_laserscan_->time_increment = (scan_time_ / (float)(cyg_driver::DATA_LENGTH_2D - 1));
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

    double angle_step_2d = static_cast<double>(CygLiDARD1::Sensor::AngleIncremet2D);
    float point_angle_var_2D = 0.0;
    float tempX_2D, tempY_2D;

    for (int i = 0; i < cyg_driver::DATA_LENGTH_2D; i++)
    {
        // Reverse data order of the array
        int data_idx = (cyg_driver::DATA_LENGTH_2D - 1 - i);

        float actual_distance = (Distance2D_[data_idx]);

        float point_angle_2D = (float)(((-HORIZONTAL_ANGLE / 2)+ point_angle_var_2D) * CygLiDARD1::Util::ToRadian);
        point_angle_var_2D += angle_step_2d;

        float actualX = (sin(point_angle_2D) * actual_distance);
        float actualY = (cos(point_angle_2D) * actual_distance);

        tempX_2D = actualX;
        tempY_2D = actualY;

        float rotation_angle = -90.0f * CygLiDARD1::Util::ToRadian;

        actualX = (tempX_2D * cos(rotation_angle)) + (tempY_2D * -sin(rotation_angle));
        actualY = (tempX_2D * sin(rotation_angle)) + (tempY_2D * cos(rotation_angle));

        message_point_2d_->points[i].x = actualX * MM2M;
        message_point_2d_->points[i].y = -actualY * MM2M;
        message_point_2d_->points[i].z = 0.0;

        if (Distance2D_[data_idx] < (float)(CygLiDARD1::Distance::Mode2D::Maximum_Depth_2D))
        {
            message_point_2d_->points[i].r = 255;
            message_point_2d_->points[i].g = 255;
            message_point_2d_->points[i].b = 0;
            message_point_2d_->points[i].a = 255;
        }
        else
        {
            // Turn data invisible when it's greater than the maximum
            message_point_2d_->points[i].a = 0;
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

    float position_x, position_y, position_z;
    for (int buffer_index = 0; buffer_index < CygLiDARD1::Sensor::Height * CygLiDARD1::Sensor::Width; buffer_index++)
    {
        uint16_t distance = Distance3D_[buffer_index];

        if(distance < CygLiDARD1::Distance::Mode3D::Maximum_Depth_3D)
        {
            if(PointCloud.calcPointCloud(distance, buffer_index, position_x, position_y, position_z) == eCalculationStatus::SUCCESS)
            {
                message_point_3d_->points[buffer_index].x =  position_z * MM2M;
                message_point_3d_->points[buffer_index].y = -position_x * MM2M;
                message_point_3d_->points[buffer_index].z = -position_y * MM2M;
                uint32_t color_change_with_height = colorRGB.color_map[((int)position_y / 2) % colorRGB.color_map.size()];
                message_point_3d_->points[buffer_index].rgb = *reinterpret_cast<float*>(&color_change_with_height);
                message_point_3d_->points[buffer_index].a = 255;
            }
            else
            {
                message_point_3d_->points[buffer_index].x = 0;
                message_point_3d_->points[buffer_index].y = 0;
                message_point_3d_->points[buffer_index].z = 0;
                message_point_3d_->points[buffer_index].rgb = 0;
                message_point_3d_->points[buffer_index].a = 0;
            }
        }
        else
        {
                message_point_3d_->points[buffer_index].x = 0;
                message_point_3d_->points[buffer_index].y = 0;
                message_point_3d_->points[buffer_index].z = 0;
                message_point_3d_->points[buffer_index].rgb = 0;
                message_point_3d_->points[buffer_index].a = 0;
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
    priv_nh.param<std::string>("frame_id", frame_id, "laser_link");
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

        ROS_INFO("[PACKET UPDATED] PULSE DURATION : %d", duration_value);
        serial_port.requestDurationControl(static_cast<eRunMode>(run_mode), duration_mode, duration_value);
        ros::Duration(1.0).sleep();
        // sleep for a sec, by the duration

        ROS_INFO("[PACKET UPDATED] FREQUENCY CH.%d", frequency_channel);
        serial_port.requestFrequencyChannel(frequency_channel);

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
