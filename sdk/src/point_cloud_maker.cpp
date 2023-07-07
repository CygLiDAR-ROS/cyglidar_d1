#include "point_cloud_maker.h"

PointCloudMaker::PointCloudMaker(float *point_x_, float *point_y_, float *point_z_, const int32_t table_total_size_)
{
	this->table_total_size = table_total_size_;
	this->table_x = point_x_;
	this->table_y = point_y_;
	this->table_z = point_z_;
}

// centerpoint_offset : change centerpoint by using offset. (unit : pixel)
void PointCloudMaker::initLensTransform(const float sensor_point_size_mm_, const uint32_t camera_width_, const uint32_t camera_height_,
										const float center_point_offset_x_, const float center_point_offset_y_)
{
	number_of_columns = camera_width_;
	number_of_rows = camera_height_;

	int row0 = 1 - (number_of_rows / 2) + center_point_offset_x_;
	int col0 = 1 - (number_of_columns / 2) + center_point_offset_y_;

	for (y = 0, r = row0; y < number_of_rows; r++, y++)
	{
		max_check = 0.0f;

		for (x = 0, c = col0; x < number_of_columns; c++, x++)
		{
			column = static_cast<float>(c) - 0.5f;
			row = static_cast<float>(r) - 0.5f;

			angle_grad = getAngle(column, row, sensor_point_size_mm_);

			if (angle_grad > max_check) max_check = angle_grad;

			angle_rad = angle_grad * Constant_D1::Util::ToRadian;

			float rp  = sqrtf((column * column) + (row * row));
			float rUA = sinf(angle_rad);

			table_x[x + (y * Constant_D1::Sensor::Width)] = column * rUA / rp;
			table_y[x + (y * Constant_D1::Sensor::Width)] = row * rUA / rp;
			table_z[x + (y * Constant_D1::Sensor::Width)] = cosf(angle_rad);
		}
	}
}

eCalculationStatus PointCloudMaker::calcPointCloud(const uint16_t raw_distance_, const int32_t buffer_index_,
												   float &point_position_x_, float &point_position_y_, float &point_position_z_)
{
	if (buffer_index_ >= table_total_size) return eCalculationStatus::FAIL;

	float float_distance = static_cast<float>(raw_distance_);

	point_position_x_ = float_distance * table_x[buffer_index_];
	point_position_y_ = float_distance * table_y[buffer_index_];
	point_position_z_ = float_distance * table_z[buffer_index_];

	return eCalculationStatus::SUCCESS;
}

// linear interpolation function
float PointCloudMaker::interpolate(const float x_in, const float x0, const float y0, const float x1, const float y1)
{
	if (fabs(x1 - x0) < std::numeric_limits<float>::epsilon()) return y0;

	return ((x_in - x0) * (y1 - y0) / (x1 - x0)) + y0;
}

float PointCloudMaker::getAngle(const float x_, const float y_, const float sensor_point_size_mm_)
{
	float radius = sensor_point_size_mm_ * sqrtf((x_ * x_) + (y_ * y_));
	float alfa_grad = 0;

	for (int i = 1; i < camera_lens_buffer_size; i++)
	{
		if (radius >= real_image_height[i - 1] && radius <= real_image_height[i])
		{
			alfa_grad = interpolate(radius, real_image_height[i - 1], lens_angle[i - 1], real_image_height[i], lens_angle[i]);
		}
	}
	return alfa_grad;
}

void PointCloudMaker::initColorMap()
{
	uint8_t r_setup = 0;
	uint8_t g_setup = 0;
	uint8_t b_setup = 255;

    // Iterate for-loop of adding RGB value to an array
	for (int i = 0; i < 3; i++)
	{
		for (int color_count = 0; color_count < 256; color_count++)
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

			uint32_t rgb_setup = ((uint32_t)r_setup << 16 | (uint32_t)g_setup << 8 | (uint32_t)b_setup);
			color_map.push_back(rgb_setup);
		}
	}
}
