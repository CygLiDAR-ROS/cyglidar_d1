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
            int buffer_index;
            uint16_t raw_distance;
            float angle_increment_steps;
            float point_2d_angle, point_2d_angle_variable;

            float camera_coordinate_x, camera_coordinate_y;


            void publishScanLaser(std::string frame_id_, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr &publisher_laserscan_,
                                  rclcpp::Time start_time_, uint16_t *distance_buffer_2d_);

            void publishPoint2D(std::string frame_id_, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher_point_2d_,
                                uint16_t *distance_buffer_2d_);
    };
}

#endif
