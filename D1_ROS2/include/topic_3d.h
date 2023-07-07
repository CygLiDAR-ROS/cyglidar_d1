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

using Image = sensor_msgs::msg::Image;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class Topic3D
{
    public:
        void initPublisher(rclcpp::Publisher<Image>::SharedPtr _publisher_image, rclcpp::Publisher<PointCloud2>::SharedPtr _publisher_point_3d);

        void assignImage(std::string _frame_id);
        void publishScanImage(uint16_t *_distance_buffer_3d);

        void assignPCL3D(std::string _frame_id);
        void mappingPointCloud3D(uint16_t *_distance_buffer_3d);
        void publishPoint3D(uint16_t *_distance_buffer_3d);

    private:
        PointCloudMaker *pointcloud_maker;

        rclcpp::Publisher<Image>::SharedPtr       publisher_image;
        rclcpp::Publisher<PointCloud2>::SharedPtr publisher_point_3d;

        Image       message_image;
        PointCloud2 message_point_cloud_3d;
        pcl_XYZRGBA pcl_3d;

        const uint16_t bad_point = 0;        
        uint16_t  buffer_index;
        uint16_t  raw_distance;
        uint16_t* depth_data;
        uint16_t  color_level;
        uint16_t  total_color_number = 510; //color map array's total size is 510
        uint32_t  rgb_setup;
        float color_gap;
        float camera_coordinate_x, camera_coordinate_y, camera_coordinate_z;
        float param_x[Constant_D1::Sensor::numPixel];
        float param_y[Constant_D1::Sensor::numPixel];
        float param_z[Constant_D1::Sensor::numPixel];
};

#endif
