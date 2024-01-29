#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "CYG_Constant.h"
#include "CYG_Driver.h"
#include "CYG_Distortion.h"

using Image = sensor_msgs::msg::Image;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class Topic3D
{
    public:
        Topic3D();
        virtual ~Topic3D();

        void initPublisher(rclcpp::Publisher<Image>::SharedPtr _publisher_image, rclcpp::Publisher<PointCloud2>::SharedPtr _publisher_point_3d);

        void assignImage(const std::string &_frame_id);
        void assignPCL3D(const std::string &_frame_id);

        void publishDepthFlatImage(uint16_t* _distance_buffer_3d);
        void publishDepthPointCloud3D(uint16_t* _distance_buffer_3d);

        void updateColorConfig(uint8_t _color_mode, std::string &_notice);

    private:
		void initColorMap();

        rclcpp::Publisher<Image>::SharedPtr       publisher_image;
        rclcpp::Publisher<PointCloud2>::SharedPtr publisher_point_3d;

        std::shared_ptr<CYG_Distortion> cyg_distortion;
        std::shared_ptr<Image>          message_image;
        std::shared_ptr<PointCloud2>    message_point_cloud_3d;
        std::shared_ptr<pcl_XYZRGBA>    pcl_3d;

        std::vector<ColorCode_t> color_map;
        uint8_t r_setup, g_setup, b_setup;
		uint8_t color_array;
        uint8_t color_mode;

        uint16_t total_color_number;
        uint16_t buffer_index;
        uint16_t raw_distance;
        uint16_t color_level;
        uint16_t image_step;
        uint32_t rgb_setup;

        float color_gap;
        float real_world_coordinate_x, real_world_coordinate_y, real_world_coordinate_z;
};
