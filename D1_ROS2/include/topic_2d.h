#ifndef __D1_2D_TOPIC_H
#define __D1_2D_TOPIC_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "d_series_constant.h"
#include "cyglidar_driver.h"

using LaserScan = sensor_msgs::msg::LaserScan;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class Topic2D
{
    public:
        void initPublisher(rclcpp::Publisher<LaserScan>::SharedPtr _publisher_laserscan, rclcpp::Publisher<PointCloud2>::SharedPtr _publisher_point_2d);
        void assignLaserScan(std::string _frame_id);
        void publishScanLaser(rclcpp::Time _start_time, uint16_t* _distance_buffer_2d);

        void assignPCL2D(std::string _frame_id);
        void mappingPointCloud2D(uint16_t* _distance_buffer_2d);
        void publishPoint2D(uint16_t* _distance_buffer_2d);

    private:
        rclcpp::Publisher<LaserScan>::SharedPtr   publisher_laserscan;
        rclcpp::Publisher<PointCloud2>::SharedPtr publisher_point_2d;
        std::shared_ptr<LaserScan>   message_laserscan;
        std::shared_ptr<PointCloud2> message_point_cloud_2d;
        std::shared_ptr<pcl_XYZRGBA> pcl_2d;

        uint8_t buffer_index;
        uint16_t raw_distance;
        float camera_coordinate_x, camera_coordinate_y;
        const float angle_increment_steps = Constant_D1::Sensor::AngleIncremet2D;
        const float HORIZONTAL_ANGLE = 120.0f;
};


#endif
