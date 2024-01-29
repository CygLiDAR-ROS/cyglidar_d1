#pragma once

#include <cstdint>
#include <limits>
#include <cmath>

#include "CYG_Constant.h"

class CYG_Distortion
{
	public:
		void initLensTransform(const float _sensor_point_size_mm, const uint8_t _sensor_width, const uint8_t _sensor_height,
							   const float _center_point_offset_x, const float _center_point_offset_y);

		void transformPixel(uint16_t _buffer_index, uint16_t _source_origin_z,
							float &_point_position_x, float &_point_position_y, float &_point_position_z);

	private:
		float interpolate(const float x_in, const float x0, const float y0, const float x1, const float y1);
		float getAngle(const float _x, const float _y, const float _sensor_point_size_mm);

		float table_x[D1_Const::IMAGE_WIDTH * D1_Const::IMAGE_HEIGHT];
        float table_y[D1_Const::IMAGE_WIDTH * D1_Const::IMAGE_HEIGHT];
        float table_z[D1_Const::IMAGE_WIDTH * D1_Const::IMAGE_HEIGHT];

		int x, y, r, c;
		int number_of_columns;
		int number_of_rows;

		float max_check;
		float angle_grad, angle_rad;
		float x_over_hypotenuse;
		float y_over_hypotenuse;
		float focal_length_over_hypotenuse;
};

// Distortion Type : F - Tan(Theta)
const uint8_t DISTORTION_TABLE_SIZE = 101;

// lens_angle units are Degree.
const float LENS_ANGLE[DISTORTION_TABLE_SIZE] =
{
	0.000000f,  0.649278f,  1.298556f,  1.947834f,  2.597113f,  3.246391f,
	3.895669f,  4.544947f,  5.194225f,  5.843503f,  6.492782f,  7.142060f,
	7.791338f,  8.440616f,  9.089894f,  9.739172f,  10.388451f, 11.037729f,
	11.687007f, 12.336285f, 12.985563f, 13.634841f, 14.284120f, 14.933398f,
	15.582676f, 16.231954f,	16.881232f, 17.530510f, 18.179789f, 18.829067f,
	19.478345f, 20.127623f,	20.776901f, 21.426179f, 22.075457f, 22.724736f,
	23.374014f, 24.023292f,	24.672570f, 25.321848f, 25.971126f, 26.620405f,
	27.269683f, 27.918961f,	28.568239f, 29.217517f, 29.866795f, 30.516074f,
	31.165352f, 31.814630f, 32.463908f, 33.113186f, 33.762464f, 34.411743f,
	35.061021f, 35.710299f, 36.359577f, 37.008855f, 37.658133f, 38.307412f,
	38.956690f, 39.605968f,	40.255246f, 40.904524f, 41.553802f, 42.203080f,
	42.852359f, 43.501637f, 44.150915f, 44.800193f, 45.449471f, 46.098749f,
	46.748028f, 47.397306f, 48.046584f, 48.695862f, 49.345140f, 49.994418f,
	50.643697f, 51.292975f, 51.942253f, 52.591531f, 53.240809f, 53.890087f,
	54.539366f, 55.188644f, 55.837922f, 56.487200f, 57.136478f, 57.785756f,
	58.435035f, 59.084313f, 59.733591f, 60.382869f, 61.032147f, 61.681425f,
	62.330703f, 62.979982f, 63.629260f, 64.278538f, 64.927816f
};

// real_image_height units are Millimeters.
const float REAL_IMAGE_HEIGHT[DISTORTION_TABLE_SIZE] =
{
	0.000000f, 0.009109f, 0.018221f, 0.027340f, 0.036469f, 0.045612f, 0.054770f,
	0.063949f, 0.073151f, 0.082379f, 0.091637f, 0.100928f, 0.110256f, 0.119622f,
	0.129032f, 0.138487f, 0.147992f, 0.157548f, 0.167160f, 0.176831f, 0.186563f,
	0.196359f, 0.206223f, 0.216158f, 0.226167f, 0.236252f, 0.246417f, 0.256665f,
	0.266998f, 0.277420f, 0.287933f, 0.298541f, 0.309246f, 0.320052f, 0.330961f,
	0.341977f, 0.353103f, 0.364342f, 0.375698f, 0.387174f, 0.398773f, 0.410500f,
	0.422359f, 0.434354f, 0.446488f, 0.458768f, 0.471197f, 0.483782f, 0.496527f,
	0.509438f, 0.522522f, 0.535786f, 0.549236f, 0.562880f, 0.576726f, 0.590783f,
	0.605060f, 0.619565f, 0.634311f, 0.649306f, 0.664563f, 0.680093f, 0.695909f,
	0.712025f, 0.728453f, 0.745209f, 0.762308f, 0.779765f, 0.797597f, 0.815821f,
	0.834455f, 0.853516f, 0.873023f, 0.892996f, 0.913455f, 0.934419f, 0.955910f,
	0.977950f, 1.000560f, 1.023764f, 1.047585f, 1.072049f, 1.097183f, 1.123014f,
	1.149573f, 1.176892f, 1.205008f, 1.233960f, 1.263789f, 1.294543f, 1.326272f,
	1.359031f, 1.392875f, 1.427865f, 1.464057f, 1.501507f, 1.540263f, 1.580362f,
	1.621823f, 1.664646f, 1.708800f
};
