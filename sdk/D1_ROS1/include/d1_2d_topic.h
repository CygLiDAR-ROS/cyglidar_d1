#include "d_series_constant.h"
#include "cyglidar_driver.h"
#include "serial.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace D1
{
    class Topic_2D
    {
        public:
            void publishScanLaser(ros::Publisher publisher_laserscan_, sensor_msgs::LaserScan::Ptr message_laserscan_, std::string frame_id_,
                                  ros::Time start_, double scan_time_, uint16_t *distance_buffer_2d_);

            void publishPoint2D(ros::Publisher publisher_point_2d_, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr message_point_2d_,
                                 std::string frame_id_, uint16_t *distance_buffer_2d_);
    };
}

