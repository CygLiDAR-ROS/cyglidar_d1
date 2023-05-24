#ifndef __D1_3D_TOPIC_H
#define __D1_3D_TOPIC_H

#include "d_series_constant.h"
#include "cyglidar_driver.h"
#include "serial.h"
#include "point_cloud_maker.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace D1
{
    class Topic_3D
    {
        public:
            uint16_t buffer_index;
            uint16_t raw_distance;
            uint16_t* depth_data;
            const uint16_t bad_point = 0;
            
            uint16_t new_color_3d, color_level;
            uint16_t total_color_number = 511; //color map array's total size is 511
            uint32_t rgb_setup;
            float color_gap;

            float camera_coordinate_x, camera_coordinate_y, camera_coordinate_z;            

            void publishScanImage(std::string frame_id, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_, uint16_t *distance_buffer_3d_);
            void assignImageData(std::string frame_id_, sensor_msgs::msg::Image &distance_image_);

            void publishPoint3D(std::string frame_id_, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher_point_3d_,
                                PointCloudMaker &pointcloud_maker, uint16_t *distance_buffer_3d_);
            void asssignPointCloud3DPosition(pcl::PointCloud<pcl::PointXYZRGBA> &pointcloud_3d_, PointCloudMaker &pointcloud_maker_, uint16_t *distance_buffer_3d_);
    };
}

#endif
