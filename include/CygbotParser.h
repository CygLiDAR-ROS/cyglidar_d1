#ifndef __CYGBOTPARSER_H
#define __CYGBOTPARSER_H
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define DEBUG

#define TRUE 1
#define FALSE 0

// Packet position
#define POS_CYGBOT_HEADER	0
#define POS_DEVICE			1
#define POS_ID				2
#define POS_LENGTH_1		3
#define POS_LENGTH_2		4
#define POS_PAYLOAD_HEADER	5

// Payload position
#define PAYLOAD_POS_HEADER 0
#define PAYLOAD_POS_DATA 1


// Header1
#define NORMAL_MODE 0x5A

// Header2
/*
* ---Code List---
* BeetleBot = 0x11
* CygLidar = 0x77
*/
#define PRODUCT_CODE 0x77

// Header3
#define DEFAULT_ID 0xFF

#define HEADER_LENGTH_SIZE 5	// header1, header2, header3, length1, length2�� ����Ʈ ũ�� ��

	enum CMDMode {
		Idle = 0,
		NormalMode
	};

	enum PacketCheckList {
		Header1 = 0,
		Header2,
		Header3,
		Length1,
		Length2,
		Payload_Header,
		Payload_Data,
		CheckSum
	};

	typedef struct Packet_Payload
	{
		uint8_t Header;
		uint8_t *Data;
	}Payload;

	typedef struct CygPacket
	{
		uint8_t Header1;
		uint8_t Header2;
		uint8_t Header3;
		uint16_t Length;
		Payload *Payload;
		uint8_t Checksum;
	}Packet;

	uint8_t CygParser(uint8_t *CmdBuff, uint8_t data);
	uint8_t Make_Packet(uint8_t *CmdBuff, Packet *pk, Payload *pl);
	uint8_t Calc_Checksum(uint8_t *buff, int buffSize);

	//__declspec(dllexport) Payload;
	//__declspec(dllexport) Packet;
	//__declspec(dllexport) uint8_t CygParser(uint8_t data);
	//__declspec(dllexport) uint8_t Make_Packet(Packet *pk, Payload *pl);
	//__declspec(dllexport) uint8_t Calc_Checksum(uint8_t *buff, int buffSize);

#ifdef	__cplusplus
}
#endif


#endif /* __CYGBOTPARSER_H */
