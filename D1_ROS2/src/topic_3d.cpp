#include "topic_3d.h"

using namespace Constant_D1;

void Topic3D::initPublisher(rclcpp::Publisher<Image>::SharedPtr _publisher_image, rclcpp::Publisher<PointCloud2>::SharedPtr _publisher_point_3d)
{
    publisher_image    = _publisher_image;
    publisher_point_3d = _publisher_point_3d;
}

void Topic3D::assignImage(std::string _frame_id)
{
    message_image.header.stamp = rclcpp::Clock().now();
    message_image.header.frame_id = _frame_id;
    message_image.height = static_cast<uint32_t>(Sensor::Height);
    message_image.width  = static_cast<uint32_t>(Sensor::Width);
    message_image.encoding = sensor_msgs::image_encodings::MONO16;
    message_image.step = message_image.width * sizeof(uint16_t);
    message_image.is_bigendian = false;
    message_image.data.resize(message_image.height * message_image.step);
}

void Topic3D::publishScanImage(uint16_t *_distance_buffer_3d)
{
    depth_data = reinterpret_cast<uint16_t*>(&message_image.data[0]);

    for (buffer_index = 0; buffer_index < message_image.height * message_image.width; buffer_index++)
    {
        if (_distance_buffer_3d[buffer_index] < Distance::Mode3D::Maximum_Depth_3D)
        {
            depth_data[buffer_index] = _distance_buffer_3d[buffer_index];
        }
        else
        {
            depth_data[buffer_index] = bad_point;
        }
    }

    publisher_image->publish(message_image);
}

void Topic3D::assignPCL3D(std::string frame_id_)
{
    pcl_3d.header.frame_id = frame_id_;
    pcl_3d.is_dense = false;
    pcl_3d.width  = Sensor::Width;
    pcl_3d.height = Sensor::Height;
    pcl_3d.points.resize(cyg_driver::DATA_LENGTH_3D);

    pointcloud_maker = new PointCloudMaker(param_x, param_y, param_z, Sensor::numPixel);

    // Call the following function so as to store colors to draw 3D data
    pointcloud_maker->initColorMap();
    pointcloud_maker->initLensTransform(Sensor::PixelRealSize, Sensor::Width, Sensor::Height,
                                       Parameter::OffsetCenterPoint_x, Parameter::OffsetCenterPoint_y);
}

void Topic3D::mappingPointCloud3D(uint16_t *_distance_buffer_3d)
{
    pcl_conversions::toPCL(rclcpp::Clock().now(), pcl_3d.header.stamp);

    color_gap = Distance::Mode3D::Maximum_Depth_3D / total_color_number;

    for (buffer_index = 0; buffer_index < Sensor::Height * Sensor::Width; buffer_index++)
    {
        raw_distance = _distance_buffer_3d[buffer_index];

        //Color arrangement changes with distance
        color_level = (int)((float)raw_distance / color_gap) >= total_color_number ? (total_color_number - 1) : (int)raw_distance / color_gap;

        if(raw_distance < Distance::Mode3D::Maximum_Depth_3D)
        {
            pointcloud_maker->calcPointCloud(raw_distance, buffer_index, camera_coordinate_x, camera_coordinate_y, camera_coordinate_z);

            pcl_3d.points[buffer_index].x = camera_coordinate_z * Util::MM_To_M;
            pcl_3d.points[buffer_index].y = -camera_coordinate_x * Util::MM_To_M;
            pcl_3d.points[buffer_index].z = -camera_coordinate_y * Util::MM_To_M;
            rgb_setup = pointcloud_maker->color_map[color_level];
            pcl_3d.points[buffer_index].rgb = *reinterpret_cast<float *>(&rgb_setup);
            pcl_3d.points[buffer_index].a = 255;
        }
        else
        {
            pcl_3d.points[buffer_index].x = 0;
            pcl_3d.points[buffer_index].y = 0;
            pcl_3d.points[buffer_index].z = 0;
            pcl_3d.points[buffer_index].rgba = 0;
        }
    }
}

void Topic3D::publishPoint3D(uint16_t *_distance_buffer_3d)
{
    mappingPointCloud3D(_distance_buffer_3d);

    pcl::toROSMsg(pcl_3d, message_point_cloud_3d); //change type from pointcloud(pcl) to ROS message(PointCloud2)
    publisher_point_3d->publish(message_point_cloud_3d);
}

