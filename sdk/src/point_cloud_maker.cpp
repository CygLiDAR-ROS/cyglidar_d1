#include "point_cloud_maker.h"

PointCloudMaker::PointCloudMaker(float* _point_x, float* _point_y, float *_point_z, const int32_t _table_total_size)
{
	this->table_total_size = _table_total_size;
	this->table_x = _point_x;
	this->table_y = _point_y;
	this->table_z = _point_z;
}

// centerpoint_offset : change centerpoint by using offset. (unit : pixel)
void PointCloudMaker::initLensTransform(const float _sensor_point_size_mm, const uint32_t _camera_width, const uint32_t _camera_height,
										const float _center_point_offset_x, const float _center_point_offset_y)
{
	number_of_columns = _camera_width;
	number_of_rows = _camera_height;

	int row0 = 1 - (number_of_rows / 2) + _center_point_offset_x;
	int col0 = 1 - (number_of_columns / 2) + _center_point_offset_y;

	for (y = 0, r = row0; y < number_of_rows; r++, y++)
	{
		max_check = 0.0f;

		for (x = 0, c = col0; x < number_of_columns; c++, x++)
		{
			column = static_cast<float>(c) - 0.5f;
			row    = static_cast<float>(r) - 0.5f;

			angle_grad = getAngle(column, row, _sensor_point_size_mm);

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

eCalculationStatus PointCloudMaker::calcPointCloud(const uint16_t _raw_distance, const int32_t _buffer_index,
												   float &_point_position_x, float &_point_position_y, float &_point_position_z)
{
	if (_buffer_index >= table_total_size) return eCalculationStatus::FAIL;

	_point_position_x = static_cast<float>(_raw_distance) * table_x[_buffer_index];
	_point_position_y = static_cast<float>(_raw_distance) * table_y[_buffer_index];
	_point_position_z = static_cast<float>(_raw_distance) * table_z[_buffer_index];

	return eCalculationStatus::SUCCESS;
}

// linear interpolation function
float PointCloudMaker::interpolate(const float x_in, const float x0, const float y0, const float x1, const float y1)
{
	if (fabs(x1 - x0) < std::numeric_limits<float>::epsilon()) return y0;

	return ((x_in - x0) * (y1 - y0) / (x1 - x0)) + y0;
}

float PointCloudMaker::getAngle(const float _x, const float _y, const float _sensor_point_size_mm)
{
	float radius = _sensor_point_size_mm * sqrtf((_x * _x) + (_y * _y));
	float alfa_grad = 0;

	for (uint8_t i = 1; i < CAMERA_LENS_BUFFER_SIZE; i++)
	{
		if (radius >= real_image_height[i - 1] && radius <= real_image_height[i])
		{
			alfa_grad = interpolate(radius, real_image_height[i - 1], lens_angle[i - 1], real_image_height[i], lens_angle[i]);
		}
	}

	return alfa_grad;
}

void PointCloudMaker::initColorMap(uint8_t color_mode)
{
	r_setup = 0;
	g_setup = 0;
	b_setup = 255;

	if (color_mode == MODE_HUE)
	{
		color_array = 5;
	}
	else if (color_mode == MODE_RGB || color_mode == MODE_GRAY)
	{
		color_array = 2;
	}

    // Iterate for-loop of adding RGB value to an array
	for (uint8_t i = 0; i < color_array; i++)
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

			//uint32_t rgb_setup = ((uint32_t)r_setup << 16 | (uint32_t)g_setup << 8 | (uint32_t)b_setup);
			color_map.push_back({r_setup, g_setup, b_setup});
		}
	}
}
