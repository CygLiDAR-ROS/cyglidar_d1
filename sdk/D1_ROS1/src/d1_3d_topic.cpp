#include "d1_3d_topic.h"

namespace D1
{
    void Topic_3D::publishScanImage(ros::Publisher publisher_image_, sensor_msgs::Image::Ptr message_image_,
                                    std::string frame_id_, uint16_t *payload_data_buffer_3d)
    {
        message_image_->header.stamp = ros::Time::now();
        message_image_->header.frame_id = frame_id_;
        message_image_->height = static_cast<uint32_t>(CygLiDARD1::Sensor::Height);
        message_image_->width = static_cast<uint32_t>(CygLiDARD1::Sensor::Width);
        message_image_->encoding = sensor_msgs::image_encodings::MONO16;
        message_image_->step = message_image_->width * sizeof(uint16_t);
        message_image_->is_bigendian = 0;
        message_image_->data.resize(message_image_->height * message_image_->step);

        int index;
        uint16_t bad_point = 0;
        uint16_t* depth_data = reinterpret_cast<uint16_t*>(&message_image_->data[0]);

        for (int y = 0; y < message_image_->height; y++)
        {
            for (int x = 0; x < message_image_->width; x++)
            {
                index = x + (y * message_image_->width);

                if (payload_data_buffer_3d[index] < CygLiDARD1::Distance::Mode3D::Maximum_Depth_3D)
                {
                    depth_data[index] = payload_data_buffer_3d[index];
                }
                else depth_data[index] = bad_point;
            }
        }
        publisher_image_.publish(message_image_);
    }

    void Topic_3D::publishPoint3D(ros::Publisher publisher_point_3d_, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr message_point_3d_, std::string frame_id_,
                                  PointCloudMaker &PointCloud, std::vector<uint32_t> color_map, uint16_t *distance_buffer_3d_)
    {
        message_point_3d_->header.frame_id = frame_id_;
        message_point_3d_->is_dense = false;
        message_point_3d_->width  = CygLiDARD1::Sensor::Width;
        message_point_3d_->height = CygLiDARD1::Sensor::Height;
        message_point_3d_->points.resize(cyg_driver::DATA_LENGTH_3D);

        uint16_t raw_distance;
        float world_coordinate_x, world_coordinate_y, world_coordinate_z;

        for (int buffer_index = 0; buffer_index < CygLiDARD1::Sensor::Height * CygLiDARD1::Sensor::Width; buffer_index++)
        {
            raw_distance = distance_buffer_3d_[buffer_index];

            if(raw_distance < CygLiDARD1::Distance::Mode3D::Maximum_Depth_3D)
            {
                PointCloud.calcPointCloud(raw_distance, buffer_index, world_coordinate_x, world_coordinate_y, world_coordinate_z);

                message_point_3d_->points[buffer_index].x =  world_coordinate_z * MM2M;
                message_point_3d_->points[buffer_index].y = -world_coordinate_x * MM2M;
                message_point_3d_->points[buffer_index].z = -world_coordinate_y * MM2M;
                uint32_t color_change_with_height = color_map[((int)world_coordinate_y / 2) % color_map.size()];
                message_point_3d_->points[buffer_index].rgb = *reinterpret_cast<float*>(&color_change_with_height);
                message_point_3d_->points[buffer_index].a = 255;
            }
            else
            {
                message_point_3d_->points[buffer_index].x = 0;
                message_point_3d_->points[buffer_index].y = 0;
                message_point_3d_->points[buffer_index].z = 0;
                message_point_3d_->points[buffer_index].rgb = 0;
                message_point_3d_->points[buffer_index].a = 0;
            }
        }
        pcl_conversions::toPCL(ros::Time::now(), message_point_3d_->header.stamp);
        publisher_point_3d_.publish(message_point_3d_);
    }
}
