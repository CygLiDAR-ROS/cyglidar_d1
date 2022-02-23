/*
 * Constants_CygLiDAR_D1.cpp
 *
 *  Created on: 2022. 2. 16.
 *      Author: choi
 */

#ifndef CONSTANTS_CYGLIDAR_D1_H_
#define CONSTANTS_CYGLIDAR_D1_H_

#include <stdint.h>

typedef struct ColorCodeRGB
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
}ColorCode_t;

namespace CygLiDARD1
{
	namespace Sensor
	{
		const float PixelRealSize = 0.02f;				// unit : mm
		const float HorizontalAngle = 120.0f;		// unit : degree
		const float AngleIncremet2D = 0.75f;		// unit : degree
		const int DataSize2D	= 161;	// 	161 => HorizontalAngle / AngleIncremet2D 
		const int32_t Width = 160;					// unit : pixel	
		const int32_t Height = 60;					// unit : pixel	
		const int32_t numPixel = Width * Height;	// unit : pixel	
	}

	namespace Parameter
	{
		const float OffsetCenterPoint_x = 0.0f;		// unit : pixel
		const float OffsetCenterPoint_y = 0.0f;		// unit : pixel	
	}

	namespace Distance
	{
		namespace Mode2D
		{
			const uint16_t Minimum_Depth_2D = 100;		// unit : mm
			const uint16_t Maximum_Depth_2D = 10000;	// unit : mm

			namespace ErrorCode
			{
				const uint16_t Invalid = 16000;
				const uint16_t LowAmplitude = 16001;
				const uint16_t ADCOverflow = 16002;
				const uint16_t Saturation = 16003;
				const uint16_t BadPixel = 16004;
				const uint16_t LowDCS = 16005;
				const uint16_t Interference = 16007;
				const uint16_t EdgeFilterd = 16008;
			}
		}

		namespace Mode3D
		{
			const uint16_t Maximum_Depth_3D = 3000;		// unit : mm

			namespace ErrorCode
			{
				const uint16_t Invalid = 4080;
				const uint16_t LowAmplitude = 4081;
				const uint16_t ADCOverflow = 4082;
				const uint16_t Saturation = 4083;
				const uint16_t BadPixel = 4084;
				const uint16_t LowDCS = 4085;
				const uint16_t Interference = 4086;
				const uint16_t EdgeFilterd = 4087;
			}
		}
	}

	namespace Color
	{
		const ColorCode_t ADCOverflow = { 173,216,230 };
		const ColorCode_t Saturation = { 128,0,128 };
	}

	namespace Util
	{
		const float PI = 3.14159265f;	// pi
		const float MM_To_M = 0.001f;		// length mm to m
		const float ToRadian = PI / 180.0f;
	}

	namespace Command
	{
		namespace Header
		{
			const uint8_t HeaderTotalSize = 5;	// {header1, header2, header3, payloadlength1, payloadlengt2}
			const uint8_t Header1 = 0x5A;
			const uint8_t Header2 = 0x77;
			const uint8_t Header3 = 0xFF;
		}

		namespace Payload
		{
			namespace Run
			{
				const uint32_t PayloadTotalSize = 2;
				namespace PayloadHeader
				{
					const uint8_t Run2D 	= 0x01;
					const uint8_t Run3D 	= 0x08;
					const uint8_t RunDual 	= 0x07;
				}
				namespace PayloadData
				{
					const uint8_t data = 0x00;
				}
			}

			namespace Stop
			{
				const uint32_t PayloadTotalSize = 2;
				namespace PayloadHeader
				{
					const uint8_t Stop = 0x02;	
				}
				namespace PayloadData
				{
					const uint8_t data = 0x00;
				}
			}

			namespace Frequency
			{
				const uint32_t PayloadTotalSize = 2;
				namespace PayloadHeader
				{
					const uint8_t SetFreqeuncy = 0x0F;	
				}
				namespace PayloadData
				{
					const uint8_t data = 0x00;
				}
			}	

			namespace Duration
			{
				const uint32_t PayloadTotalSize = 3;
				const uint16_t MaximumDurationValue = 10000;	// unit : us
				namespace PayloadHeader
				{
					const uint8_t Duration = 0x0C;	
				}
				namespace PayloadData
				{
					const uint8_t data = 0x00;
				}
			}
		}
	}
}

#endif