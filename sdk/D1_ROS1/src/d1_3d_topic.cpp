#include "d1_3d_topic.h"

namespace D1
{
    using namespace CygLiDARD1;

    void Topic_3D::publishScanImage(std::string frame_id, ros::Publisher publisher_image_, uint16_t *payload_data_buffer_3d)
    {
        sensor_msgs::Image message_image;

        assignImageData(frame_id, message_image);

        depth_data = reinterpret_cast<uint16_t*>(&message_image.data[0]);

        for (buffer_index = 0; buffer_index < message_image.height * message_image.width; buffer_index++)
        {
            if (distance_buffer_3d_[buffer_index] < Distance::Mode3D::Maximum_Depth_3D)
                depth_data[buffer_index] = distance_buffer_3d_[buffer_index];
            else
                depth_data[buffer_index] = bad_point;
        }
        publisher_image_.publish(message_image);
    }
    
    void Topic_3D::assignImageData(std::string frame_id_, sensor_msgs::Image &message_image_)
    {
        message_image_.header.stamp = ros::Time::now();
        message_image_.header.frame_id = frame_id_;
        message_image_.height = static_cast<uint32_t>(Sensor::Height);
        message_image_.width  = static_cast<uint32_t>(Sensor::Width);
        message_image_.encoding = sensor_msgs::image_encodings::MONO16;
        message_image_.step = message_image_.width * sizeof(uint16_t);
        message_image_.is_bigendian = 0;
        message_image_.data.resize(message_image_.height * message_image_.step);
    }

    void Topic_3D::publishPoint3D(std::string frame_id_, ros::Publisher publisher_point_3d_, PointCloudMaker &pointcloud_maker, uint16_t *distance_buffer_3d_)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr message_point_3d(new pcl::PointCloud<pcl::PointXYZRGBA>);

        message_point_3d->header.frame_id = frame_id_;
        message_point_3d->is_dense = false;
        message_point_3d->width  = Sensor::Width;
        message_point_3d->height = Sensor::Height;
        message_point_3d->points.resize(cyg_driver::DATA_LENGTH_3D);

        for (buffer_index = 0; buffer_index < Sensor::Height * Sensor::Width; buffer_index++)
        {
            raw_distance = distance_buffer_3d_[buffer_index];

            if(raw_distance < Distance::Mode3D::Maximum_Depth_3D)
            {
                pointcloud_maker.calcPointCloud(raw_distance, buffer_index, camera_coordinate_x, camera_coordinate_y, camera_coordinate_z);

                message_point_3d->points[buffer_index].x =  camera_coordinate_z * MM2M;
                message_point_3d->points[buffer_index].y = -camera_coordinate_x * MM2M;
                message_point_3d->points[buffer_index].z = -camera_coordinate_y * MM2M;
                color_change_with_height = pointcloud_maker.color_map[((int)camera_coordinate_y / 2) % pointcloud_maker.color_map.size()];
                message_point_3d->points[buffer_index].rgb = *reinterpret_cast<float*>(&color_change_with_height);
                message_point_3d->points[buffer_index].a = 255;
            }
            else
            {
                message_point_3d->points[buffer_index].x = 0;
                message_point_3d->points[buffer_index].y = 0;
                message_point_3d->points[buffer_index].z = 0;
                message_point_3d->points[buffer_index].rgb = 0;
                message_point_3d->points[buffer_index].a = 0;
            }
        }
        pcl_conversions::toPCL(ros::Time::now(), message_point_3d->header.stamp);
        publisher_point_3d_.publish(message_point_3d);
    }
}
