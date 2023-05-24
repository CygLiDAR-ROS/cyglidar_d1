#include "d_series_constant.h"
#include "cyglidar_driver.h"
#include "serial.h"
#include "point_cloud_maker.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace D1
{
    class Topic_3D
    {
        public:
            uint16_t buffer_index;
            uint16_t raw_distance;
            uint16_t *depth_data;
            const uint16_t bad_point = 0;
            
            uint16_t new_color_3d, color_level;
            uint16_t total_color_number = 511; //color map array's total size is 511
            uint32_t rgb_setup;
            float color_gap;

            float camera_coordinate_x, camera_coordinate_y, camera_coordinate_z;            

            void publishScanImage(std::string frame_id, ros::Publisher publisher_image_, uint16_t *distance_buffer_3d_);
            void assignImageData(std::string frame_id_, sensor_msgs::Image &message_image_);

            void publishPoint3D(std::string frame_id_, ros::Publisher publisher_point_3d_, PointCloudMaker &pointcloud_maker, uint16_t *distance_buffer_3d_);
            void asssignPointCloud3DPosition(pcl::PointCloud<pcl::PointXYZRGBA> &point_3d_, PointCloudMaker &pointcloud_maker_, uint16_t *distance_buffer_3d_);
    };
}


