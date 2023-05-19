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
            int buffer_index;
            uint16_t raw_distance;
            uint16_t* depth_data;
            uint32_t color_change_with_height;

            float camera_coordinate_x, camera_coordinate_y, camera_coordinate_z;
            const uint16_t bad_point = 0;

            void publishScanImage(std::string frame_id_, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_,
                                  uint16_t *payload_data_buffer_3d);

            void publishPoint3D(std::string frame_id_, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher_point_3d_,
                                PointCloudMaker &pointcloud_maker, uint16_t *distance_buffer_3d_);
    };
}

#endif
