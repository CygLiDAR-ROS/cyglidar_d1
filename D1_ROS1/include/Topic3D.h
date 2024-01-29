#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "CYG_Constant.h"
#include "CYG_Driver.h"
#include "CYG_Distortion.h"

using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class Topic3D
{
    public:
        Topic3D();
        virtual ~Topic3D();

        void initPublisher(ros::Publisher _publisher_image, ros::Publisher _publisher_point_3d);

        void mappingPointCloud3D(uint16_t* _distance_buffer_3d);

        void assignPCL3D(const std::string &_frame_id);
        void publishPoint3D(uint16_t* _distance_buffer_3d);

        void assignImage(const std::string &_frame_id);
        void publishScanImage(uint16_t* _distance_buffer_3d);

    private:
        void initColorMap();

        ros::Publisher publisher_image;
        ros::Publisher publisher_point_3d;

        std::shared_ptr<CYG_Distortion>           cyg_distortion;
        std::shared_ptr<sensor_msgs::PointCloud2> message_point_cloud_3d;
        std::shared_ptr<sensor_msgs::Image>       message_image;
        std::shared_ptr<pcl_XYZRGBA>              pcl_3d;

        std::vector<ColorCode_t> color_map;
        uint8_t r_setup, g_setup, b_setup;

        uint16_t  buffer_index;
        uint16_t  raw_distance;
        uint16_t  color_level;
        uint16_t  total_color_number = 510;
        uint16_t* depth_data;
        uint32_t  rgb_setup;

        float color_gap;
        float real_world_coordinate_x, real_world_coordinate_y, real_world_coordinate_z;

        const uint16_t BAD_POINT = 0;
};
