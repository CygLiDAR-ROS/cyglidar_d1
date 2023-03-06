#include "cygparser.h"

int payload_count = 0;
int payload_size = 0;

uint8_t CPC;

enum eCommandMode command_mode = IdleMode;
enum ePacketCheckList packet_check_list = Header1;

void parsePayload(uint8_t *buffer, int size, CygPayload *payload);

extern inline void initPacket(uint8_t packet_data)
{
	if (packet_data == NORMAL_MODE)
	{
		command_mode = NormalMode;
		packet_check_list = Header2;
	}
	else
	{
		command_mode = IdleMode;
		packet_check_list = Header1;
	}
}

uint8_t makePacket(uint8_t *command_buffer, Packet *packet, CygPayload *payload)

{
	uint8_t first_bit = command_buffer[POS_CYGBOT_HEADER];
	uint8_t second_bit = command_buffer[POS_DEVICE];
	uint8_t third_bit = command_buffer[POS_ID];
	uint16_t payload_size = ((command_buffer[POS_LENGTH_2] << 8) & 0xff00) | (command_buffer[POS_LENGTH_1] & 0x00ff);
	uint8_t check_sum = command_buffer[payload_size + HEADER_LENGTH_SIZE];
	parsePayload(&command_buffer[POS_PAYLOAD_HEADER], payload_size, payload);

	packet->Header1 = first_bit;
	packet->Header2 = second_bit;
	packet->Header3 = third_bit;
	packet->Length = payload_size;
	packet->Payload = &command_buffer[POS_PAYLOAD_HEADER];
	packet->Checksum = check_sum;

	return 1;
}

void parsePayload(uint8_t *buffer, int size, CygPayload *payload)
{
	payload->Header = buffer[PAYLOAD_POS_HEADER];
	payload->Data = &buffer[PAYLOAD_POS_DATA];
}

//buffer_size = packet total byte (Header1 + Header2 + Header3 + Length1 + Length2 + Payload(Command) + Checksum)
uint8_t calcCheckSum(uint8_t *buffer, int buffer_size)
{
	uint8_t check_sum = 0;
	for (int i = POS_LENGTH_1; i < buffer_size - 1; i++)
	{
		check_sum ^= buffer[i];
	}
	return check_sum;
}

uint8_t CygParser(uint8_t *command_buffer, uint8_t packet_data)
{
	switch (packet_check_list)
	{
	case Header1:
		if (packet_data == NORMAL_MODE)
		{
			command_buffer[POS_CYGBOT_HEADER] = NORMAL_MODE;
			command_mode = NormalMode;
			packet_check_list = Header2;
			return 0x02;
		}
		break;
	case Header2:
		if (packet_data == PRODUCT_CODE && command_mode == NormalMode)
		{
			command_buffer[POS_DEVICE] = PRODUCT_CODE;
			packet_check_list = Header3;
		}
		else
		{
			initPacket(packet_data);
		}
		break;
	case Header3:
		command_buffer[POS_ID] = packet_data;
		packet_check_list = Length1;
		break;
	case Length1:
		command_buffer[POS_LENGTH_1] = packet_data;
		packet_check_list = Length2;
		break;
	case Length2:
		command_buffer[POS_LENGTH_2] = packet_data;
		payload_size = ((command_buffer[POS_LENGTH_2] << 8) & 0xff00) | (command_buffer[POS_LENGTH_1] & 0x00ff); // 길이 조건의 유연성도 주면 좋고? rplidar의 패킷 데이터와 비교해보자
		packet_check_list = Payload_Header;
		break;
	case Payload_Header:
		command_buffer[POS_PAYLOAD_HEADER] = packet_data;
		packet_check_list = Payload_Data;
		payload_count++;
		break;
	case Payload_Data:
		command_buffer[POS_PAYLOAD_HEADER + payload_count] = packet_data;
		payload_count++;

		if (payload_count > payload_size)
		{
			initPacket(packet_data); // packet overflow
			return 0;
		}
		if (payload_size == payload_count)
		{
			packet_check_list = CheckSum;
			payload_count = 0;
		}
		break;
	case CheckSum:
		initPacket(0);
		CPC = calcCheckSum(command_buffer, payload_size + 6);
		if (CPC == packet_data)
		{
			command_buffer[POS_PAYLOAD_HEADER + payload_size] = packet_data;
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