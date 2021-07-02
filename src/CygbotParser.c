#include "CygbotParser.h"

int PayloadCounter = 0;
int PayloadSize = 0;

uint8_t CPC;

enum CMDMode CM = Idle;	//Cmd Mode
enum PacketCheckList PCL = Header1;	//Packet Check List

void Payload_Parsing(uint8_t *buff, int size, Payload *pl);

inline void Init_Packt(uint8_t data)
{
	if (data == NORMAL_MODE)
	{
		CM = NormalMode;
		PCL = Header2;
	}
	else
	{
		CM = Idle;
		PCL = Header1;
	}
}

uint8_t Make_Packet(uint8_t *CmdBuff, Packet *pk, Payload *pl)
{
	uint8_t FirstBit = CmdBuff[POS_CYGBOT_HEADER];
	uint8_t SecondBit = CmdBuff[POS_DEVICE];
	uint8_t ThirdBit = CmdBuff[POS_ID];
	uint16_t PayloadSize = ((CmdBuff[POS_LENGTH_2] << 8) & 0xff00) | (CmdBuff[POS_LENGTH_1] & 0x00ff);
	uint8_t CheckSum = CmdBuff[PayloadSize + HEADER_LENGTH_SIZE];
	Payload_Parsing(&CmdBuff[POS_PAYLOAD_HEADER], PayloadSize, pl);

	pk->Header1 = FirstBit;
	pk->Header2 = SecondBit;
	pk->Header3 = ThirdBit;
	pk->Length = PayloadSize;
	pk->Payload = &CmdBuff[POS_PAYLOAD_HEADER];
	pk->Checksum = CheckSum;

	return TRUE;
}

void Payload_Parsing(uint8_t *buff, int size, Payload *pl)
{
	pl->Header = buff[PAYLOAD_POS_HEADER];
	pl->Data = &buff[PAYLOAD_POS_DATA];
}


//buffsize = packet total byte (header1 + header2 + header3 + length1 + length2 + PayloadCommand + checksum)
uint8_t Calc_Checksum(uint8_t *buff, int buffSize)
{
	uint8_t CheckSum = 0;
	for (int i = POS_LENGTH_1; i < buffSize - 1; i++)
	{
		CheckSum ^= buff[i];
	}
	return CheckSum;
}

uint8_t CygParser(uint8_t *CmdBuff, uint8_t data)
{
	switch (PCL)
	{
	case Header1:
		if (data == NORMAL_MODE)
		{
			CmdBuff[POS_CYGBOT_HEADER] = NORMAL_MODE;
			CM = NormalMode;
			PCL = Header2;
			return 0x02;
		}
		break;
	case Header2:
		if (data == PRODUCT_CODE && CM == NormalMode)
		{
			CmdBuff[POS_DEVICE] = PRODUCT_CODE;
			PCL = Header3;
		}
		else
		{
			Init_Packt(data);
		}
		break;
	case Header3:
		CmdBuff[POS_ID] = data;
		PCL = Length1;
		break;
	case Length1:
		CmdBuff[POS_LENGTH_1] = data;
		PCL = Length2;
		break;
	case Length2:
		CmdBuff[POS_LENGTH_2] = data;
		PayloadSize = ((CmdBuff[POS_LENGTH_2] << 8) & 0xff00) | (CmdBuff[POS_LENGTH_1] & 0x00ff);
		PCL = Payload_Header;
		break;
	case Payload_Header:
		CmdBuff[POS_PAYLOAD_HEADER] = data;
		PCL = Payload_Data;
		PayloadCounter++;
		break;
	case Payload_Data:
		CmdBuff[POS_PAYLOAD_HEADER + PayloadCounter] = data;
		PayloadCounter++;

		if (PayloadCounter > PayloadSize)
		{
			Init_Packt(data);	//packet overflow
			return FALSE;
		}
		if (PayloadSize == PayloadCounter)
		{
			PCL = CheckSum;
			PayloadCounter = 0;
		}
		break;
	case CheckSum:
		Init_Packt(0);
		CPC = Calc_Checksum(CmdBuff, PayloadSize + 6);		//Calculated Packet Checksum
		if (CPC == data)
		{
			CmdBuff[POS_PAYLOAD_HEADER + PayloadSize] = data;
			return TRUE;		//packet parsing success
		}
		break;
	default:
		Init_Packt(0);
		break;
	}
	return FALSE;
}