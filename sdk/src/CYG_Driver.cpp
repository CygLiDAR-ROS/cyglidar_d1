#include "CYG_Driver.h"

// Convert 2D, 3D rawdata to measured distance appending both separated data (LSB and MSB)
void CYG_Driver::getDistanceArray2D(uint8_t* _received_buffer_2d, uint16_t* _distance_2d)
{
    uint8_t buffer_count_2d = 0;

    for (uint16_t data_length = 0; data_length < PACKET_LENGTH_2D; data_length += 2)
    {
        data_msb = _received_buffer_2d[data_length];
        data_lsb = _received_buffer_2d[data_length + 1];

        _distance_2d[buffer_count_2d++] = static_cast<uint16_t>((data_msb << 8) | data_lsb);
    }
}

void CYG_Driver::getDistanceArray3D(uint8_t* _received_buffer_3d, uint16_t* _distance_3d)
{
    uint16_t buffer_count_3d = 0;

    for (uint16_t data_length = 0; data_length < PACKET_LENGTH_DISTANCE_3D; data_length += 3)
    {
        first  = _received_buffer_3d[data_length];
        second = _received_buffer_3d[data_length + 1];
        third  = _received_buffer_3d[data_length + 2];

        data1 = (first << 4) | (second >> 4);
        data2 = ((second & 0xf) << 8) | third;

        _distance_3d[buffer_count_3d++] = data1;
        _distance_3d[buffer_count_3d++] = data2;
    }
}

uint8_t CYG_Driver::CygParser(uint8_t* _command_buffer, uint8_t packet_data)
{
	switch (packet_check_list)
	{
		case kHeader1:
			if (packet_data == D1_Const::NORMAL_MODE)
			{
				_command_buffer[D1_Const::HEADER_START] = D1_Const::NORMAL_MODE;
				command_mode = kNormalMode;
				packet_check_list = kHeader2;

				return D1_Const::PARSING_STARTED;
			}
			break;
		case kHeader2:
			if (packet_data == D1_Const::PRODUCT_CODE && command_mode == kNormalMode)
			{
				_command_buffer[D1_Const::HEADER_DEVICE] = D1_Const::PRODUCT_CODE;
				packet_check_list = kHeader3;
			}
			else
			{
				initPacket(packet_data);
			}
			break;
		case kHeader3:
			_command_buffer[D1_Const::HEADER_ID] = packet_data;
			packet_check_list = kLength_LSB;
			break;
		case kLength_LSB:
			_command_buffer[D1_Const::LENGTH_LSB] = packet_data;
			packet_check_list = kLength_MSB;
			break;
		case kLength_MSB:
			_command_buffer[D1_Const::LENGTH_MSB] = packet_data;
			payload_size = ((_command_buffer[D1_Const::LENGTH_MSB] << 8) & 0xFF00) | (_command_buffer[D1_Const::LENGTH_LSB] & 0x00FF);
			packet_check_list = kPayload_Header;
			payload_count = 0;
			break;
		case kPayload_Header:
			_command_buffer[kPayload_Header] = packet_data;
			packet_check_list = kPayload_Data;
			payload_count++;
			break;
		case kPayload_Data:
			_command_buffer[kPayload_Header + payload_count] = packet_data;
			payload_count++;

			if (payload_count > payload_size)
			{
				initPacket(packet_data); // packet overflow
				return D1_Const::PARSING_FAILED;
			}

			if (payload_size == payload_count)
			{
				packet_check_list = kCheckSum;
				payload_count = 0;
			}
			break;
		case kCheckSum:
			initPacket(0);

			check_sum_byte = calcCheckSum(_command_buffer, payload_size + 6);

			if (check_sum_byte == packet_data)
			{
				_command_buffer[D1_Const::PAYLOAD_HEADER + payload_size] = packet_data;
				return D1_Const::CHECKSUM_PASSED;
				// Succeed packet parsing
			}
			break;
		default:
			initPacket(0);
			break;
	}

	return 0;
}

void CYG_Driver::initPacket(uint8_t _packet_data)
{
	if (_packet_data == D1_Const::NORMAL_MODE)
	{
		command_mode = kNormalMode;
		packet_check_list = kHeader2;
	}
	else
	{
		command_mode = kIdleMode;
		packet_check_list = kHeader1;
	}
}

//buffer_size = packet total byte (Header1 + Header2 + Header3 + Length1 + Length2 + Payload(Command) + Checksum)
uint8_t CYG_Driver::calcCheckSum(uint8_t* _buffer, uint16_t _buffer_size)
{
	sum = 0;

	for (uint16_t i = D1_Const::LENGTH_LSB; i < _buffer_size - 1; i++)
	{
		sum ^= _buffer[i];
	}

	return sum;
}
