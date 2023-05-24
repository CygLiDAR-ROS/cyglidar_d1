#ifndef __D1_2D_TOPIC_H
#define __D1_2D_TOPIC_H

#include "d_series_constant.h"
#include "cyglidar_driver.h"
#include "serial.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace D1
{
    class Topic_2D
    {
        public:
            uint8_t buffer_index;
            uint16_t raw_distance;
            float angle_increment_steps;
            float point_2d_angle, point_2d_angle_variable;

            float camera_coordinate_x, camera_coordinate_y;


            void publishScanLaser(std::string frame_id, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr &publisher_laserscan_,
                                  rclcpp::Time start_time, uint16_t *distance_buffer_2d_);
            void assignLaserScanData(std::string frame_id_, rclcpp::Time start_time_, sensor_msgs::msg::LaserScan &message_laserscan_);

            void publishPoint2D(std::string frame_id_, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher_point_2d_,
                                uint16_t *distance_buffer_2d_);
            void asssignPointCloud2DPosition(pcl::PointCloud<pcl::PointXYZRGBA> &pointcloud_2d_, uint16_t *distance_buffer_2d_);
    };
}

#endif
