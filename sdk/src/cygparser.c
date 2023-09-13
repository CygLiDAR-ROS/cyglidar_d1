#include "cygparser.h"

uint16_t payload_count = 0;
uint16_t payload_size  = 0;

uint8_t CPC;

enum eCommandMode command_mode = kIdleMode;
enum ePacketCheckList packet_check_list = kHeader1;

extern inline void initPacket(uint8_t packet_data)
{
	if (packet_data == NORMAL_MODE)
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
uint8_t calcCheckSum(uint8_t* buffer, int buffer_size)
{
	uint8_t check_sum = 0;

	for (int i = LENGTH_LSB; i < buffer_size - 1; i++)
	{
		check_sum ^= buffer[i];
	}

	return check_sum;
}

uint8_t CygParser(uint8_t* command_buffer, uint8_t packet_data)
{
	switch (packet_check_list)
	{
		case kHeader1:
			if (packet_data == NORMAL_MODE)
			{
				command_buffer[CYGBOT_HEADER] = NORMAL_MODE;
				command_mode = kNormalMode;
				packet_check_list = kHeader2;

				return 0x02;
			}
			break;
		case kHeader2:
			if (packet_data == PRODUCT_CODE && command_mode == kNormalMode)
			{
				command_buffer[DEVICE] = PRODUCT_CODE;
				packet_check_list = kHeader3;
			}
			else
			{
				initPacket(packet_data);
			}
			break;
		case kHeader3:
			command_buffer[ID] = packet_data;
			packet_check_list = kLength_LSB;
			break;
		case kLength_LSB:
			command_buffer[LENGTH_LSB] = packet_data;
			packet_check_list = kLength_MSB;
			break;
		case kLength_MSB:
			command_buffer[LENGTH_MSB] = packet_data;
			payload_size = ((command_buffer[LENGTH_MSB] << 8) & 0xff00) | (command_buffer[LENGTH_LSB] & 0x00ff);
			packet_check_list = kPayload_Header;
			payload_count = 0;
			break;
		case kPayload_Header:
			command_buffer[PAYLOAD_HEADER] = packet_data;
			packet_check_list = kPayload_Data;
			payload_count++;
			break;
		case kPayload_Data:
			command_buffer[PAYLOAD_HEADER + payload_count] = packet_data;
			payload_count++;

			if (payload_count > payload_size)
			{
				initPacket(packet_data); // packet overflow
				return 0;
			}

			if (payload_size == payload_count)
			{
				packet_check_list = kCheckSum;
				payload_count = 0;
			}
			break;
		case kCheckSum:
			initPacket(0);

			CPC = calcCheckSum(command_buffer, payload_size + 6);

			if (CPC == packet_data)
			{
				command_buffer[PAYLOAD_HEADER + payload_size] = packet_data;
				return 1;
				// Succeed packet parsing
			}
			break;
		default:
			initPacket(0);
			break;
	}
	return 0;
}
