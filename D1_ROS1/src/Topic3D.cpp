#include "Topic3D.h"

Topic3D::Topic3D()
{
	cyg_distortion         = std::make_shared<CYG_Distortion>();
    message_image          = std::make_shared<sensor_msgs::Image>();
    message_point_cloud_3d = std::make_shared<sensor_msgs::PointCloud2>();
}

Topic3D::~Topic3D() {}

void Topic3D::initPublisher(ros::Publisher _publisher_image, ros::Publisher _publisher_point_3d)
{
    publisher_image    = _publisher_image;
    publisher_point_3d = _publisher_point_3d;
}

void Topic3D::publishPoint3D(uint16_t* _distance_buffer_3d)
{
    pcl_conversions::toPCL(ros::Time::now(), pcl_3d->header.stamp);

    color_gap = D1_Const::DISTANCE_MAX_VALUE_3D / total_color_number;

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
            pcl_3d->points[buffer_index].rgba = 0;
        }
    }

    pcl::toROSMsg(*pcl_3d, *message_point_cloud_3d);
    publisher_point_3d.publish(*message_point_cloud_3d);
}

void Topic3D::assignPCL3D(const std::string &_frame_id)
{
    pcl_3d.reset(new pcl_XYZRGBA());

    pcl_3d->header.frame_id = _frame_id;
    pcl_3d->is_dense        = false;
    pcl_3d->width           = D1_Const::IMAGE_WIDTH;
    pcl_3d->height          = D1_Const::IMAGE_HEIGHT;
    pcl_3d->points.resize(DATA_LENGTH_3D);

    initColorMap();

    cyg_distortion->initLensTransform(ROS_Const::PIXEL_REAL_SIZE, D1_Const::IMAGE_WIDTH, D1_Const::IMAGE_HEIGHT,
                                      ROS_Const::OFFSET_CENTER_POINT_X, ROS_Const::OFFSET_CENTER_POINT_Y);
}

void Topic3D::assignImage(const std::string &_frame_id)
{
    message_image->header.frame_id = _frame_id;
    message_image->header.stamp    = ros::Time::now();
    message_image->width           = D1_Const::IMAGE_WIDTH;
    message_image->height          = D1_Const::IMAGE_HEIGHT;
    message_image->encoding        = sensor_msgs::image_encodings::MONO16;
    message_image->step            = message_image->width * sizeof(uint16_t);
    message_image->is_bigendian    = false;
    message_image->data.resize(message_image->height * message_image->step);
}

void Topic3D::publishScanImage(uint16_t *_distance_buffer_3d)
{
    depth_data = reinterpret_cast<uint16_t*>(&message_image->data[0]);

    for (buffer_index = 0; buffer_index < message_image->height * message_image->width; buffer_index++)
    {
        if (_distance_buffer_3d[buffer_index] < D1_Const::DISTANCE_MAX_VALUE_3D)
        {
            depth_data[buffer_index] = _distance_buffer_3d[buffer_index];
        }
        else
        {
            depth_data[buffer_index] = BAD_POINT;
        }
    }

    publisher_image.publish(*message_image);
}

void Topic3D::initColorMap()
{
    r_setup = 0;
    g_setup = 0;
    b_setup = 255;

    // Iterate for-loop of adding RGB value to an array
	for (uint8_t i = 0; i < 2; i++)
	{
		for (uint8_t color_count = 0; color_count < 255; color_count++)
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

			color_map.push_back({r_setup, g_setup, b_setup, 0xFF});
		}
	}
}
