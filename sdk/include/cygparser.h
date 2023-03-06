#ifndef __CYGBOTPARSER_H
#define __CYGBOTPARSER_H
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define DEBUG

// Packet position
#define POS_CYGBOT_HEADER		0
#define POS_DEVICE			1
#define POS_ID				2
#define POS_LENGTH_1			3
#define POS_LENGTH_2			4
#define POS_PAYLOAD_HEADER		5

// Payload position
#define PAYLOAD_POS_HEADER 		0
#define PAYLOAD_POS_DATA 		1

// Header1
#define NORMAL_MODE 			0x5A

// Header2
/*
* ---Code List---
* BeetleBot = 0x11
* CygLidar = 0x77
*/
#define PRODUCT_CODE 			0x77

// Header3
#define DEFAULT_ID 			0xFF

#define CHECKSUM_PASSED 		1

#define HEADER_LENGTH_SIZE  		5

	enum eCommandMode {
		IdleMode = 0,
		NormalMode
	};

	enum ePacketCheckList {
		Header1 = 0,
		Header2,
		Header3,
		Length1,
		Length2,
		Payload_Header,
		Payload_Data,
		CheckSum
	};

	typedef struct CygPayload
	{
		uint8_t Header;
		uint8_t *Data;
	}CygPayload;

	typedef struct CygPacket
	{
		uint8_t Header1;
		uint8_t Header2;
		uint8_t Header3;
		uint16_t Length;
		CygPayload *Payload;
		uint8_t Checksum;
	}Packet;

	uint8_t CygParser(uint8_t *command_buffer, uint8_t packet_data);
	uint8_t makePacket(uint8_t *command_buffer, Packet *packet, CygPayload *payload);
	uint8_t calcCheckSum(uint8_t *buffer, int buffer_size);

#ifdef	__cplusplus
}
#endif


#endif /* __CYGBOTPARSER_H */
