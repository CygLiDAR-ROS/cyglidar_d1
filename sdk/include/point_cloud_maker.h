#ifndef __POINT_CLOUD_MAKER_H
#define __POINT_CLOUD_MAKER_H

#include "d_series_camera_lens.h"
#include "d_series_constant.h"

#include <stdint.h>
#include <vector>
#include <limits>
#include <math.h>

enum eCalculationStatus
{
	FAIL,
	SUCCESS
};

class PointCloudMaker
{
	public:
		PointCloudMaker(float *point_x_, float *point_y_, float *point_z_, const int32_t table_total_size);

		void initLensTransform(const float sensor_point_size_mm_, const uint32_t camera_width_, const uint32_t camera_height_,
							   const float centerpoint_offset_x_, const float center_point_offset_y_);

		eCalculationStatus calcPointCloud(const uint16_t raw_distance_, const int32_t buffer_index_,
										  float &point_position_x_, float &point_position_y_, float &point_position_z_);

		void initColorMap();
		std::vector<uint32_t> color_map;

	private:
		float interpolate(const float x_in, const float x0, const float y0, const float x1, const float y1);
		float getAngle(const float x_, const float y_, const float sensor_point_size_mm_);

		float *table_x;
		float *table_y;
		float *table_z;
		int32_t table_total_size;

		int x, y, r, c;
		int number_of_columns;
		int number_of_rows;

		float column, row;
		float max_check;
		float angle_grad, angle_rad;
};

#endif
