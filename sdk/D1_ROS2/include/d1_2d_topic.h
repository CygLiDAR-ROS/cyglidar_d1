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
            void publishScanLaser(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr &publisher_laserscan_, sensor_msgs::msg::LaserScan::SharedPtr message_laserscan_,
                                  std::string frame_id_, rclcpp::Time start_, double scan_time_, uint16_t *distance_buffer_2d_);

            void publishPoint2D(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher_point_2d_, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud_2d_,
                                std::string frame_id_, uint16_t *distance_buffer_2d_);
    };
}
