#ifndef __POINT_CLOUD_MAKER_H
#define __POINT_CLOUD_MAKER_H

#include <cstint>
#include <math.h>
#include <vector>
#include <limits>

#include "d_series_camera_lens.h"
#include "d_series_constant.h"

enum eCalculationStatus
{
	FAIL,
	SUCCESS
};

class PointCloudMaker
{
	public:
		PointCloudMaker(float* _point_x, float* _point_y, float* _point_z, const int32_t _table_total_size);

		void initLensTransform(const float _sensor_point_size_mm, const uint32_t _camera_width, const uint32_t _camera_height,
							   const float _center_point_offset_x, const float _center_point_offset_y);

		eCalculationStatus calcPointCloud(const uint16_t _raw_distance, const int32_t _buffer_index,
										  float &_point_position_x, float &_point_position_y, float &_point_position_z);

		void initColorMap();

		std::vector<uint32_t> color_map;

	private:
		float interpolate(const float x_in, const float x0, const float y0, const float x1, const float y1);
		float getAngle(const float _x, const float _y, const float _sensor_point_size_mm);

		float* table_x;
		float* table_y;
		float* table_z;
		int32_t table_total_size;

		uint8_t r_setup, g_setup, b_setup;
		int x, y, r, c;
		int number_of_columns;
		int number_of_rows;

		float column, row;
		float max_check;
		float angle_grad, angle_rad;
};

#endif
