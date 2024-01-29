#include "Topic3D.h"

Topic3D::Topic3D()
{
	cyg_distortion         = std::make_shared<CYG_Distortion>();
    message_point_cloud_3d = std::make_shared<PointCloud2>();
    message_image          = std::make_shared<Image>();
}

Topic3D::~Topic3D() {}

void Topic3D::initPublisher(rclcpp::Publisher<Image>::SharedPtr _publisher_image, rclcpp::Publisher<PointCloud2>::SharedPtr _publisher_point_3d)
{
    publisher_image    = _publisher_image;
    publisher_point_3d = _publisher_point_3d;
}

void Topic3D::assignPCL3D(const std::string &_frame_id)
{
    pcl_3d.reset(new pcl_XYZRGBA());

    pcl_3d->header.frame_id = _frame_id;
    pcl_3d->is_dense        = false;
    pcl_3d->width           = D1_Const::IMAGE_WIDTH;
    pcl_3d->height          = D1_Const::IMAGE_HEIGHT;
    pcl_3d->points.resize(DATA_LENGTH_3D);

    cyg_distortion->initLensTransform(ROS_Const::PIXEL_REAL_SIZE, D1_Const::IMAGE_WIDTH, D1_Const::IMAGE_HEIGHT,
                                      ROS_Const::OFFSET_CENTER_POINT_X, ROS_Const::OFFSET_CENTER_POINT_Y);
}

void Topic3D::assignImage(const std::string &_frame_id)
{
    message_image->header.frame_id = _frame_id;
    message_image->header.stamp    = rclcpp::Clock().now();
    message_image->height          = D1_Const::IMAGE_HEIGHT;
    message_image->width           = D1_Const::IMAGE_WIDTH;
    message_image->encoding        = sensor_msgs::image_encodings::RGBA8;
    message_image->step            = message_image->width * sizeof(uint32_t);
    message_image->is_bigendian    = false;
    message_image->data.resize(message_image->height * message_image->step);
}

void Topic3D::publishDepthFlatImage(uint16_t* _distance_buffer_3d)
{
	for (uint8_t y = 0; y < D1_Const::IMAGE_HEIGHT; y++)
    {
        for (uint8_t x = 0; x < D1_Const::IMAGE_WIDTH; x++)
        {
            image_step = (x * sizeof(uint32_t)) + (y * D1_Const::IMAGE_WIDTH * sizeof(uint32_t));
            buffer_index = x + (y * D1_Const::IMAGE_WIDTH);

            raw_distance = _distance_buffer_3d[buffer_index];
            color_level = (int)((float)raw_distance / color_gap) >= total_color_number ? (total_color_number - 1) : (int)raw_distance / color_gap;

            if (raw_distance < D1_Const::DISTANCE_MAX_VALUE_3D)
            {
                message_image->data[image_step]     = color_map[color_level].R;
                message_image->data[image_step + 1] = color_map[color_level].G;
                message_image->data[image_step + 2] = color_map[color_level].B;
                message_image->data[image_step + 3] = color_map[color_level].A;
            }
            else if (raw_distance == D1_Const::ADC_OVERFLOW_3D)
            {
                message_image->data[image_step]     = ROS_Const::ADC_OVERFLOW_COLOR.R;
                message_image->data[image_step + 1] = ROS_Const::ADC_OVERFLOW_COLOR.G;
                message_image->data[image_step + 2] = ROS_Const::ADC_OVERFLOW_COLOR.B;
                message_image->data[image_step + 3] = ROS_Const::ADC_OVERFLOW_COLOR.A;
            }
            else if (raw_distance == D1_Const::SATURATION_3D)
            {
                message_image->data[image_step]     = ROS_Const::SATURATION_COLOR.R;
                message_image->data[image_step + 1] = ROS_Const::SATURATION_COLOR.G;
                message_image->data[image_step + 2] = ROS_Const::SATURATION_COLOR.B;
                message_image->data[image_step + 3] = ROS_Const::SATURATION_COLOR.A;
            }
            else
            {
                message_image->data[image_step]     = ROS_Const::NONE_COLOR.R;
                message_image->data[image_step + 1] = ROS_Const::NONE_COLOR.G;
                message_image->data[image_step + 2] = ROS_Const::NONE_COLOR.B;
                message_image->data[image_step + 3] = ROS_Const::NONE_COLOR.A;
            }
        }
    }

    publisher_image->publish(*message_image);
}

void Topic3D::publishDepthPointCloud3D(uint16_t* _distance_buffer_3d)
{
    pcl_conversions::toPCL(rclcpp::Clock().now(), pcl_3d->header.stamp);

	for (buffer_index = 0; buffer_index < D1_Const::IMAGE_WIDTH * D1_Const::IMAGE_HEIGHT; buffer_index++)
    {
        raw_distance = _distance_buffer_3d[buffer_index];

        if(raw_distance < D1_Const::DISTANCE_MAX_VALUE_3D)
        {
            cyg_distortion->transformPixel(buffer_index, raw_distance, real_world_coordinate_x, real_world_coordinate_y, real_world_coordinate_z);

            pcl_3d->points[buffer_index].x =  real_world_coordinate_z * ROS_Const::MM2M;
            pcl_3d->points[buffer_index].y = -real_world_coordinate_x * ROS_Const::MM2M;
            pcl_3d->points[buffer_index].z = -real_world_coordinate_y * ROS_Const::MM2M;

            color_level = (int)((float)raw_distance / color_gap) >= total_color_number ? (total_color_number - 1) : (int)raw_distance / color_gap;

            rgb_setup = ((uint32_t)color_map[color_level].R << 16 | (uint32_t)color_map[color_level].G << 8 | (uint32_t)color_map[color_level].B);
            pcl_3d->points[buffer_index].rgb = *reinterpret_cast<float *>(&rgb_setup);
            pcl_3d->points[buffer_index].a = color_map[color_level].A;
        }
        else
        {
            pcl_3d->points[buffer_index].x = 0;
            pcl_3d->points[buffer_index].y = 0;
            pcl_3d->points[buffer_index].z = 0;
            pcl_3d->points[buffer_index].a = 0;
        }
    }

    pcl::toROSMsg(*pcl_3d, *message_point_cloud_3d); //change type from pointcloud(pcl) to ROS message(PointCloud2)
    publisher_point_3d->publish(*message_point_cloud_3d);
}

void Topic3D::updateColorConfig(uint8_t _color_mode, std::string &_notice)
{
    color_mode = _color_mode;

    // Call the following function so as to store colors to draw 3D data
    initColorMap();

    if (color_mode == ROS_Const::MODE_HUE)
    {
        _notice = "HUE MODE";
    }
    else if (_color_mode == ROS_Const::MODE_RGB)
    {
        _notice = "RGB MODE";
    }
    else if (_color_mode == ROS_Const::MODE_GRAY)
    {
        _notice = "GRAY MODE";
    }

	total_color_number = color_map.size();
    color_gap = D1_Const::DISTANCE_MAX_VALUE_3D / total_color_number;
}

void Topic3D::initColorMap()
{
    r_setup = 0;
    g_setup = 0;

	if (color_mode == ROS_Const::MODE_HUE)
	{
		color_array = 5;
		b_setup = 255;
	}
	else if (color_mode == ROS_Const::MODE_RGB)
	{
		color_array = 2;
		b_setup = 255;
	}
	else if (color_mode == ROS_Const::MODE_GRAY)
	{
		color_array = 1;
		b_setup = 0;
	}

    // Iterate for-loop of adding RGB value to an array
	for (uint8_t i = 0; i < color_array; i++)
	{
		for (uint8_t color_count = 0; color_count < 255; color_count++)
		{
			if (color_mode == ROS_Const::MODE_GRAY)
			{
				switch (i)
				{
					case 0:
						r_setup++;
						g_setup++;
						b_setup++;
						break;
				}
			}
			else
			{
				switch (i)
				{
					case 0: // BLUE -> YELLOW
					case 3:
						r_setup++;
						g_setup++;
						b_setup--;
						break;
					case 1: // YELLOW -> RED
					case 4:
						g_setup--;
						break;
					case 2: // RED -> BLUE
						r_setup--;
						b_setup++;
						break;
				}
			}

			color_map.push_back({r_setup, g_setup, b_setup, 0xFF});
		}
	}
}
