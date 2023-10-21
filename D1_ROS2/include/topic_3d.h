#ifndef __D1_3D_TOPIC_H
#define __D1_3D_TOPIC_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "d_series_constant.h"
#include "cyglidar_driver.h"
#include "point_cloud_maker.h"

using Image = sensor_msgs::msg::Image;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;
using namespace Constant_D1;

class Topic3D
{
    public:
        void initPublisher(rclcpp::Publisher<Image>::SharedPtr _publisher_image, rclcpp::Publisher<PointCloud2>::SharedPtr _publisher_point_3d);

        void mappingPointCloud3D(uint16_t* _distance_buffer_3d, uint16_t _min_display, uint16_t _max_display);

        void assignPCL3D(const std::string &_frame_id);
        void publishPoint3D();

        void assignImage(const std::string &_frame_id);
        void publishScanImage();

        void updateColorConfig(uint8_t _color_mode, std::string &_notice);

    private:
        void convertColorRGB(ColorCode_t _color_array, bool _enable);
        void checkDisplayResolution(uint16_t _min_display, uint16_t _max_display);

        rclcpp::Publisher<Image>::SharedPtr       publisher_image;
        rclcpp::Publisher<PointCloud2>::SharedPtr publisher_point_3d;

        std::shared_ptr<PointCloudMaker> pointcloud_maker;
        std::shared_ptr<PointCloud2>     message_point_cloud_3d;
        std::shared_ptr<Image>           message_image;
        std::shared_ptr<pcl_XYZRGBA>     pcl_3d;

        uint8_t  rgb_sum;
        uint8_t  color_mode;
        uint16_t buffer_index;
        uint16_t raw_distance;
        uint16_t color_level;
        uint16_t total_color_number;
        uint16_t min_display, max_display;
        uint32_t rgb_setup;

        float color_gap;
        float camera_coordinate_x, camera_coordinate_y, camera_coordinate_z;
        float param_x[Sensor::numPixel];
        float param_y[Sensor::numPixel];
        float param_z[Sensor::numPixel];

        ColorCode_t adc_overflow = Color::ADCOverflow;
        ColorCode_t saturation   = Color::Saturation;
        ColorCode_t none_point   = Color::None;
};

#endif
