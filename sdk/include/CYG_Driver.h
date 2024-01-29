#pragma once

#include <cstdint>

#include "CYG_Constant.h"

const uint16_t DATA_LENGTH_2D = 161;
const uint16_t DATA_LENGTH_3D = D1_Const::IMAGE_WIDTH * D1_Const::IMAGE_HEIGHT;

const uint16_t PACKET_LENGTH_2D = DATA_LENGTH_2D * sizeof(uint16_t);
const uint16_t PACKET_LENGTH_DISTANCE_3D = DATA_LENGTH_3D * 1.5;

class CYG_Driver
{
    enum eCommandMode
	{
		kIdleMode = 0,
		kNormalMode
	};

	enum ePacketCheckList
	{
		kHeader1 = 0,
		kHeader2,
		kHeader3,
		kLength_LSB,
		kLength_MSB,
		kPayload_Header,
		kPayload_Data,
		kCheckSum
	};

    public:
        uint8_t CygParser(uint8_t* _command_buffer, uint8_t packet_data);
        void getDistanceArray2D(uint8_t* _received_buffer_2d, uint16_t* _distance_2d);
        void getDistanceArray3D(uint8_t* _received_buffer_3d, uint16_t* _distance_3d);

    private:
        void initPacket(uint8_t _packet_data);
		uint8_t calcCheckSum(uint8_t* _buffer, uint16_t _buffer_size);

		enum eCommandMode     command_mode      = kIdleMode;
		enum ePacketCheckList packet_check_list = kHeader1;

		uint8_t  check_sum_byte;
		uint8_t  sum;
		uint16_t payload_count;
		uint16_t payload_size;

        uint8_t  data_msb, data_lsb;
        uint8_t  first, second, third;
        uint16_t data1, data2;
};
