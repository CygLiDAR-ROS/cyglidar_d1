/*
 * PointCloudMaker.cpp
 *
 *  Created on: 2022. 2. 16.
 *      Author: choi
 */
#include "PointCloudMaker.h"
#include <limits>
#include <math.h>
#include "CameraLens.h"
#include "Constants_CygLiDAR_D1.h"

PointCloudMaker::PointCloudMaker(float *pbuff_x, float *pbuff_y, float *pbuff_z, const int32_t TableTotalSize)
{
	this->table_totalSize = TableTotalSize;
	this->table_x = pbuff_x;
	this->table_y = pbuff_y;
	this->table_z = pbuff_z;
}

eCalculationStatus PointCloudMaker::calcPointCloud(const uint16_t distance, const int32_t index, float& output_x, float& output_y, float& output_z)
{
	if (index >= table_totalSize) return eCalculationStatus::FAIL;

	float fDistance = static_cast<float>(distance);
	output_x = fDistance * table_x[index];
	output_y = fDistance * table_y[index];
	output_z = fDistance * table_z[index];
	return eCalculationStatus::SUCCESS;
}

// centerpoint_offset : change centerpoint by using offset. (unit : pixel)
void PointCloudMaker::initLensTransform(const float sensorPointSizeMM, const uint32_t width, const uint32_t height,
	const float centerpoint_offset_X, const float centerpoint_offset_Y)
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

			float angleGrad = getAngle(c, r, sensorPointSizeMM);
			if (angleGrad > maxCheck) maxCheck = angleGrad;

			float angleRad = angleGrad * 3.14159265f / 180.0f;

			float rp = sqrtf((c * c) + (r * r));
			float rUA = sinf(angleRad);

			table_x[x + (y * CygLiDARD1::Sensor::Width)] = c * rUA / rp;
			table_y[x + (y * CygLiDARD1::Sensor::Width)] = r * rUA / rp;
			table_z[x + (y * CygLiDARD1::Sensor::Width)] = cosf(angleRad);
		}
	}
}


float PointCloudMaker::getAngle(const float x, const float y, const float sensorPointSizeMM)
{
	float radius = sensorPointSizeMM * sqrtf((x * x) + (y * y));
	float alfaGrad = 0;
	for (int i = 1; i < CameraLens_BufferSize; i++)
	{
		if (radius >= RealImageHeight[i - 1] && radius <= RealImageHeight[i])
		{
			alfaGrad = interpolate(radius, RealImageHeight[i - 1], Angle[i - 1], RealImageHeight[i], Angle[i]);
		}
	}
	return alfaGrad;
}


// linear interpolation function
float PointCloudMaker::interpolate(const float x_in, const float x0, const float y0, const float x1, const float y1)
{
	if (fabs(x1 - x0) < std::numeric_limits<float>::epsilon()) return y0;

	return ((x_in - x0) * (y1 - y0) / (x1 - x0)) + y0;
}
