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
            uint8_t buffer_index;
            uint16_t raw_distance;
            
            float camera_coordinate_x, camera_coordinate_y;

            void publishScanLaser(std::string frame_id_, ros::Publisher publisher_laserscan_, ros::Time start_time_, uint16_t *distance_buffer_2d_);
            void publishPoint2D(std::string frame_id_, ros::Publisher publisher_point_2d_, uint16_t *distance_buffer_2d_);
    };
}

