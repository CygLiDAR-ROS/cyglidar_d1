#ifndef __D1_3D_TOPIC_H
#define __D1_3D_TOPIC_H

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

class Topic3D
{
    public:
        void initPublisher(ros::Publisher _publisher_image, ros::Publisher _publisher_point_3d);

        void assignImage(std::string _frame_id);
        void publishScanImage(uint16_t *_distance_buffer_3d);

        void assignPCL3D(std::string _frame_id);
        void mappingPointCloud3D(uint16_t *_distance_buffer_3d);
        void publishPoint3D(uint16_t *_distance_buffer_3d);

    private:
        PointCloudMaker *pointcloud_maker;

        ros::Publisher publisher_image;
        ros::Publisher publisher_point_3d;

        sensor_msgs::Image message_image;
        pcl::PointCloud<pcl::PointXYZRGBA> pcl_3d;

        uint16_t  buffer_index;
        uint16_t  raw_distance;
        uint16_t *depth_data;
        uint16_t  color_level;
        uint16_t  total_color_number = 510; //color map array's total size is 510
        uint32_t  rgb_setup;

        const uint16_t bad_point = 0;

        float color_gap;
        float camera_coordinate_x, camera_coordinate_y, camera_coordinate_z;
        float param_x[Constant_D1::Sensor::numPixel];
        float param_y[Constant_D1::Sensor::numPixel];
        float param_z[Constant_D1::Sensor::numPixel];
};

#endif
