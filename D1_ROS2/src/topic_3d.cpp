#include "topic_3d.h"

void Topic3D::initPublisher(rclcpp::Publisher<Image>::SharedPtr _publisher_image, rclcpp::Publisher<PointCloud2>::SharedPtr _publisher_point_3d)
{
    publisher_image    = _publisher_image;
    publisher_point_3d = _publisher_point_3d;
}

void Topic3D::mappingPointCloud3D(uint16_t* _distance_buffer_3d, uint16_t _min_display, uint16_t _max_display)
{
    checkDisplayResolution(_min_display, _max_display);

    pcl_conversions::toPCL(rclcpp::Clock().now(), pcl_3d->header.stamp);

    for (buffer_index = 0; buffer_index < Sensor::Height * Sensor::Width; buffer_index++)
    {
        raw_distance = _distance_buffer_3d[buffer_index];

        if(min_display < raw_distance && raw_distance < max_display)
        {
            pointcloud_maker->calcPointCloud(raw_distance, buffer_index, camera_coordinate_x, camera_coordinate_y, camera_coordinate_z);

            pcl_3d->points[buffer_index].x =  camera_coordinate_z * Util::MM_To_M;
            pcl_3d->points[buffer_index].y = -camera_coordinate_x * Util::MM_To_M;
            pcl_3d->points[buffer_index].z = -camera_coordinate_y * Util::MM_To_M;

            //Color arrangement changes with distance
            color_level = (int)((float)raw_distance / color_gap) >= total_color_number ? (total_color_number - 1) : (int)raw_distance / color_gap;
            convertColorRGB(pointcloud_maker->color_map[color_level], true);
        }
        else if (raw_distance == Distance::Mode3D::ErrorCode::ADCOverflow)
        {
            convertColorRGB(adc_overflow, false);
        }
        else if (raw_distance == Distance::Mode3D::ErrorCode::Saturation)
        {
            convertColorRGB(saturation, false);
        }
        else
        {
            pcl_3d->points[buffer_index].x = 0;
            pcl_3d->points[buffer_index].y = 0;
            pcl_3d->points[buffer_index].z = 0;
            convertColorRGB(none_point, false);
        }
    }
}

void Topic3D::assignPCL3D(const std::string &_frame_id)
{
    message_point_cloud_3d = std::make_shared<PointCloud2>();
    pcl_3d.reset(new pcl_XYZRGBA());

    pcl_3d->header.frame_id = _frame_id;
    pcl_3d->is_dense = false;
    pcl_3d->width  = Sensor::Width;
    pcl_3d->height = Sensor::Height;
    pcl_3d->points.resize(DATA_LENGTH_3D);

    pointcloud_maker = std::make_shared<PointCloudMaker>(param_x, param_y, param_z, Sensor::numPixel);

    pointcloud_maker->initLensTransform(Sensor::PixelRealSize, Sensor::Width, Sensor::Height,
                                        Parameter::OffsetCenterPoint_x, Parameter::OffsetCenterPoint_y);
}

void Topic3D::publishPoint3D()
{
    pcl::toROSMsg(*pcl_3d, *message_point_cloud_3d); //change type from pointcloud(pcl) to ROS message(PointCloud2)
    publisher_point_3d->publish(*message_point_cloud_3d);
}

void Topic3D::assignImage(const std::string &_frame_id)
{
    message_image = std::make_shared<Image>();

    message_image->header.stamp = rclcpp::Clock().now();
    message_image->header.frame_id = _frame_id;
    message_image->height = static_cast<uint32_t>(Sensor::Height);
    message_image->width  = static_cast<uint32_t>(Sensor::Width);
    message_image->encoding = sensor_msgs::image_encodings::RGBA8;
    message_image->step = message_image->width * sizeof(uint32_t);
    message_image->is_bigendian = false;
    message_image->data.resize(message_image->height * message_image->step);
}

void Topic3D::publishScanImage()
{
    pcl::toROSMsg(*pcl_3d, *message_image);
    publisher_image->publish(*message_image);
}

void Topic3D::updateColorConfig(uint8_t _color_mode, std::string &_notice)
{
    color_mode = _color_mode;

    // Call the following function so as to store colors to draw 3D data
    pointcloud_maker->initColorMap(color_mode);

    if (color_mode == MODE_HUE)
    {
        total_color_number = 1275;
        _notice = "HUE MODE";
    }
    else if (_color_mode == MODE_RGB)
    {
        total_color_number = 510;
        _notice = "RGB MODE";
    }
    else if (_color_mode == MODE_GRAY)
    {
        total_color_number = 510;
        _notice = "GRAY MODE";
    }

    color_gap = Distance::Mode3D::Maximum_Depth_3D / total_color_number;
}

void Topic3D::convertColorRGB(ColorCode_t _color_array, bool _enable)
{
    if (color_mode == MODE_GRAY && _enable == true)
    {
        rgb_sum = (_color_array.R + _color_array.G + _color_array.B) / 3 - 1;
        rgb_setup = ((uint32_t)rgb_sum << 16 | (uint32_t)rgb_sum << 8 | (uint32_t)rgb_sum);
        pcl_3d->points[buffer_index].rgb = *reinterpret_cast<float *>(&rgb_setup);
    }
    else
    {
        rgb_setup = ((uint32_t)_color_array.R << 16 | (uint32_t)_color_array.G << 8 | (uint32_t)_color_array.B);
        pcl_3d->points[buffer_index].rgb = *reinterpret_cast<float *>(&rgb_setup);
    }

    if (_enable == true)
    {
        pcl_3d->points[buffer_index].a = 255;
    }
    else if (_enable == false)
    {
        pcl_3d->points[buffer_index].a = 0;
    }
}

void Topic3D::checkDisplayResolution(uint16_t _min_display, uint16_t _max_display)
{
    min_display = _min_display;
    max_display = _max_display;

    if (min_display > 3000) min_display = 0;

    if (max_display > 3000) max_display = 3000;

    if (min_display > max_display)
    {
        uint16_t temp = min_display;
        min_display  = max_display;
        max_display  = temp;
    }
}
