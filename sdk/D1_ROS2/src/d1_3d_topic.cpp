#include "d1_3d_topic.h"

namespace D1
{
    using namespace CygLiDARD1;

    void Topic_3D::publishScanImage(std::string frame_id, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_, uint16_t *distance_buffer_3d_)
    {
        sensor_msgs::msg::Image message_image;

        assignImageData(frame_id, message_image);

        depth_data = reinterpret_cast<uint16_t*>(&message_image.data[0]);

        for (buffer_index = 0; buffer_index < message_image.height * message_image.width; buffer_index++)
        {
            if (distance_buffer_3d_[buffer_index] < Distance::Mode3D::Maximum_Depth_3D)
                depth_data[buffer_index] = distance_buffer_3d_[buffer_index];
            else
                depth_data[buffer_index] = bad_point;
        }
        publisher_image_->publish(message_image);
    }

    void Topic_3D::assignImageData(std::string frame_id_, sensor_msgs::msg::Image &message_image_)
    {
        message_image_.header.stamp = rclcpp::Clock().now();
        message_image_.header.frame_id = frame_id_;
        message_image_.height = static_cast<uint32_t>(Sensor::Height);
        message_image_.width  = static_cast<uint32_t>(Sensor::Width);
        message_image_.encoding = sensor_msgs::image_encodings::MONO16;
        message_image_.step = message_image_.width * sizeof(uint16_t);
        message_image_.is_bigendian = false;
        message_image_.data.resize(message_image_.height * message_image_.step);
    }

    void Topic_3D::publishPoint3D(std::string frame_id_, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher_point_3d_,
                                  PointCloudMaker &pointcloud_maker, uint16_t *distance_buffer_3d_)
    {
        pcl::PointCloud<pcl::PointXYZRGBA> pointcloud_3d;

        pointcloud_3d.header.frame_id = frame_id_;
        pointcloud_3d.is_dense = false;
        pointcloud_3d.width  = Sensor::Width;
        pointcloud_3d.height = Sensor::Height;
        pointcloud_3d.points.resize(cyg_driver::DATA_LENGTH_3D);

        asssignPointCloud3DPosition(pointcloud_3d, pointcloud_maker, distance_buffer_3d_);

        pcl_conversions::toPCL(rclcpp::Clock().now(), pointcloud_3d.header.stamp);

        sensor_msgs::msg::PointCloud2 message_point_cloud_3d;
        pcl::toROSMsg(pointcloud_3d, message_point_cloud_3d);
        publisher_point_3d_->publish(message_point_cloud_3d);
    }

    void Topic_3D::asssignPointCloud3DPosition(pcl::PointCloud<pcl::PointXYZRGBA> &pointcloud_3d_, PointCloudMaker &pointcloud_maker_, uint16_t *distance_buffer_3d_)
    {
        color_gap = Distance::Mode3D::Maximum_Depth_3D / total_color_number;

        for (buffer_index = 0; buffer_index < Sensor::Height * Sensor::Width; buffer_index++)
        {
            raw_distance = distance_buffer_3d_[buffer_index];

            //Color arrangement changes with distance
            new_color_3d = (int)((float)raw_distance / color_gap) >= total_color_number ? (total_color_number - 1) : (int)raw_distance / color_gap;
            color_level = total_color_number - new_color_3d;

            if(raw_distance < Distance::Mode3D::Maximum_Depth_3D)
            {
                pointcloud_maker_.calcPointCloud(raw_distance, buffer_index, camera_coordinate_x, camera_coordinate_y, camera_coordinate_z);

                pointcloud_3d_.points[buffer_index].x = camera_coordinate_z * MM2M;
                pointcloud_3d_.points[buffer_index].y = -camera_coordinate_x * MM2M;
                pointcloud_3d_.points[buffer_index].z = -camera_coordinate_y * MM2M;
                rgb_setup = pointcloud_maker_.color_map[color_level];
                pointcloud_3d_.points[buffer_index].rgb = *reinterpret_cast<float *>(&rgb_setup);
                pointcloud_3d_.points[buffer_index].a = 255;
            }
            else
            {
                pointcloud_3d_.points[buffer_index].x = 0;
                pointcloud_3d_.points[buffer_index].y = 0;
                pointcloud_3d_.points[buffer_index].z = 0;
                pointcloud_3d_.points[buffer_index].rgba = 0;
            }
        }
    }
}
