#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "CYG_Constant.h"
#include "CYG_Driver.h"

using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class Topic2D
{
    public:
        void initPublisher(ros::Publisher _publisher_laserscan, ros::Publisher _publisher_point_2d);

        void applyPointCloud2D(uint16_t* _distance_buffer_2d);

        void assignPCL2D(const std::string &_frame_id);
        void publishPoint2D();

        void assignLaserScan(const std::string &_frame_id);
        void publishScanLaser(ros::Time _start_time, uint16_t* _distance_buffer_2d);

    private:
        ros::Publisher publisher_laserscan;
        ros::Publisher publisher_point_2d;

        std::shared_ptr<sensor_msgs::LaserScan>   message_laserscan;
        std::shared_ptr<sensor_msgs::PointCloud2> message_point_cloud_2d;
        std::shared_ptr<pcl_XYZRGBA>              pcl_2d;

        uint8_t buffer_index;
        uint16_t raw_distance;
        float point_2d_angle;
        float point_2d_angle_variable;
        float real_world_coordinate_x, real_world_coordinate_y;
};
