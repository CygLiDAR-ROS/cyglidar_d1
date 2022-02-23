/*
 * PointCloudMaker.h
 *
 *  Created on: 2022. 2. 16.
 *      Author: choi
 */

#ifndef POINTCLOUDMAKER_POINTCLOUDMAKER_H_
#define POINTCLOUDMAKER_POINTCLOUDMAKER_H_
#include "stdint.h"

enum eCalculationStatus
{
	FAIL,
	SUCCESS
};

class PointCloudMaker
{
public:
	PointCloudMaker(float *pbuff_x, float *pbuff_y, float *pbuff_z, const int32_t TableTotalSize);
	void initLensTransform(const float sensorPointSizeMM, const uint32_t width, const uint32_t height,
		const float centerpoint_offset_X, const float centerpoint_offset_Y);
	eCalculationStatus calcPointCloud(const uint16_t distance, const int32_t index, float& output_x, float& output_y, float& output_z);
private:
	float interpolate(const float x_in, const float x0, const float y0, const float x1, const float y1);
	float getAngle(const float x, const float y, const float sensorPointSizeMM);

	float *table_x;
	float *table_y;
	float *table_z;
	int32_t table_totalSize;
};

#endif /* POINTCLOUDMAKER_POINTCLOUDMAKER_H_ */