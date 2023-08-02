#ifndef __CYGBOTPARSER_H
#define __CYGBOTPARSER_H
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define DEBUG

// Packet position
#define CYGBOT_HEADER		0
#define DEVICE				1
#define ID					2
#define LENGTH_LSB			3
#define LENGTH_MSB			4
#define PAYLOAD_HEADER		5
#define PAYLOAD_DATA    	6

// Header1
#define NORMAL_MODE 		0x5A
// Header2
#define PRODUCT_CODE 		0x77
// Header3
#define DEFAULT_ID 			0xFF

#define CHECKSUM_PASSED 	1

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

uint8_t CygParser(uint8_t* command_buffer, uint8_t packet_data);
uint8_t calcCheckSum(uint8_t* buffer, int buffer_size);

#ifdef	__cplusplus
}
#endif


#endif /* __CYGBOTPARSER_H */
