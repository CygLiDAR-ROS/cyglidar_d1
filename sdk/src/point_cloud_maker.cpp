#include <limits>
#include <math.h>
#include "point_cloud_maker.h"
#include "d_series_camera_lens.h"
#include "d_series_constant.h"

PointCloudMaker::PointCloudMaker(float *pbuff_x, float *pbuff_y, float *pbuff_z, const int32_t table_total_size_)
{
	this->table_total_size = table_total_size_;
	this->table_x = pbuff_x;
	this->table_y = pbuff_y;
	this->table_z = pbuff_z;
}

// centerpoint_offset : change centerpoint by using offset. (unit : pixel)
void PointCloudMaker::initLensTransform(const float sensor_point_size_MM, const uint32_t width, const uint32_t height, const float centerpoint_offset_X, const float centerpoint_offset_Y)
{
	int x, y, row, col;
	int numCols = width;
	int numRows = height;

	int r0 = 1 - (numRows / 2) + centerpoint_offset_X;
	int c0 = 1 - (numCols / 2) + centerpoint_offset_Y;

	float maxCheck = 0;

	for (y = 0, row = r0; y < numRows; row++, y++)
	{
		maxCheck = 0;
		for (x = 0, col = c0; x < numCols; col++, x++)
		{
			float c = static_cast<float>(col) - 0.5f;
			float r = static_cast<float>(row) - 0.5f;

			float angleGrad = getAngle(c, r, sensor_point_size_MM);
			if (angleGrad > maxCheck) maxCheck = angleGrad;

			float angleRad = angleGrad * 3.14159265f / 180.0f;

			float rp  = sqrtf((c * c) + (r * r));
			float rUA = sinf(angleRad);

			table_x[x + (y * CygLiDARD1::Sensor::Width)] = c * rUA / rp;
			table_y[x + (y * CygLiDARD1::Sensor::Width)] = r * rUA / rp;
			table_z[x + (y * CygLiDARD1::Sensor::Width)] = cosf(angleRad);
		}
	}
}

eCalculationStatus PointCloudMaker::calcPointCloud(const uint16_t distance, const int32_t index, float& output_x, float& output_y, float& output_z)
{
	if (index >= table_total_size) return eCalculationStatus::FAIL;

	float fDistance = static_cast<float>(distance);
	output_y = fDistance * table_y[index];
	output_x = fDistance * table_x[index];
	output_z = fDistance * table_z[index];
	return eCalculationStatus::SUCCESS;
}

// linear interpolation function
float PointCloudMaker::interpolate(const float x_in, const float x0, const float y0, const float x1, const float y1)
{
	if (fabs(x1 - x0) < std::numeric_limits<float>::epsilon()) return y0;

	return ((x_in - x0) * (y1 - y0) / (x1 - x0)) + y0;
}

float PointCloudMaker::getAngle(const float x, const float y, const float sensor_point_size_MM)
{
	float radius = sensor_point_size_MM * sqrtf((x * x) + (y * y));
	float alfaGrad = 0;
	for (int i = 1; i < camera_lens_buffer_size; i++)
	{
		if (radius >= real_image_height[i - 1] && radius <= real_image_height[i])
		{
			alfaGrad = interpolate(radius, real_image_height[i - 1], lens_angle[i - 1], real_image_height[i], lens_angle[i]);
		}
	}
	return alfaGrad;
}

void ColorRGB::initColorMap()
{
	uint8_t r_setup = 255;
	uint8_t g_setup = 0;
	uint8_t b_setup = 0;

    // Iterate for-loop of adding RGB value to an array
	for (int i = 0; i < 3; i++)
	{
		for (int colorCount = 0; colorCount < 256; colorCount++)
		{
			switch (i)
			{
			case 0: // RED -> YELLOW
				g_setup++;
				break;
			case 1: // YELLOW -> BLUE
				r_setup--;
				g_setup--;
				b_setup++;
				break;
			case 2: // BLUE -> RED
				r_setup++;
				b_setup--;
				break;
			}

			uint32_t rgb_setup = ((uint32_t)r_setup << 16 | (uint32_t)g_setup << 8 | (uint32_t)b_setup);
			color_map.push_back(rgb_setup);
		}
	}
}
