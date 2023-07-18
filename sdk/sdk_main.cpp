#include "serial.h"
#include "cyglidar_driver.h"

//Window
#ifdef _WIN32
	#include <windows.h>
	#include <conio.h>

	uint8_t end_of_text = 0;

	BOOL WINAPI ConsoleHandler(DWORD dwType)
	{
		if (dwType == CTRL_C_EVENT)
		{
			end_of_text = _getch();
			printf("user interrupted with ctrl-c\n");
		}
		return TRUE;
	}
#endif

//Linux
#ifdef __linux__
	#include <signal.h>

	bool signal_handle = false;

	void interruptSignal(int signal)
	{
		(void)signal;
		signal_handle = true;
	}
#endif

using namespace std::chrono_literals;

cyglidar_serial* serial_port;
boost::asio::io_service io;

void connectBoostSerial();
void disconnectBoostSerial();
void requestPacketData();
void loopCygParser();
void converData();

const uint8_t MODE_2D   = 0;
const uint8_t MODE_3D   = 1;
const uint8_t MODE_DUAL = 2;

const uint8_t PAYLOAD_HEADER = 5;
const uint8_t PAYLOAD_DATA   = 6;
const uint8_t PACKET_HEADER_2D = 0x01;
const uint8_t PACKET_HEADER_3D = 0x08;
const uint8_t PACKET_HEADER_DEV_INFO = 0x10;

std::string port;
uint32_t baud_rate;
uint8_t duration_mode;
uint16_t duration_value;
uint8_t run_mode;
uint8_t frequency_channel;

uint8_t  total_packet_data[SCAN_MAX_SIZE];
uint8_t  packet_structure[SCAN_MAX_SIZE];
uint8_t  parser_return;
uint16_t number_of_data;
uint16_t distance_buffer_2d[cyg_driver::DATA_LENGTH_2D];
uint16_t distance_buffer_3d[cyg_driver::DATA_LENGTH_3D];

int main()
{

#ifdef _WIN32
	port = "COM0";
#endif
#ifdef __linux__
    port = "/dev/ttyUSB0";
	signal(SIGINT, interruptSignal);
#endif
	baud_rate = 3000000;
	run_mode = MODE_DUAL;
	duration_mode = PULSE_AUTO;
	duration_value = 10000;
	frequency_channel = 0;

	try
	{
		connectBoostSerial();

#ifdef _WIN32
		while (true && end_of_text != 3)
		{
			if (!SetConsoleCtrlHandler((PHANDLER_ROUTINE)ConsoleHandler, TRUE))
			{
				return EXIT_FAILURE;
			}
#endif
#ifdef __linux__
		while (true && !signal_handle)
		{
#endif
			loopCygParser();
		}
		disconnectBoostSerial();
	}
	catch (std::exception &e)
	{
		printf("[D1 NODE ERROR] : %s\n", e.what());
	}

	return 0;
}

void connectBoostSerial()
{
	try
	{
		serial_port = new cyglidar_serial(port, baud_rate, io);

		requestPacketData();
	}
	catch(const boost::system::system_error &ex)
	{
		printf("[BOOST SERIAL ERROR] : %s\n", ex.what());
	}
}

void disconnectBoostSerial()
{
	serial_port->close();
    printf("[PACKET REQUEST] STOP\n");
}

void requestPacketData()
{
	serial_port->requestDeviceInfo();
	std::this_thread::sleep_for(3s);
	// sleep for 3s, by requsting the info data.

	printf("%s\n", serial_port->requestRunMode(static_cast<eRunMode>(run_mode)).c_str());

	serial_port->requestDurationControl(static_cast<eRunMode>(run_mode), duration_mode, duration_value);
	std::this_thread::sleep_for(1s);
	printf("[PACKET REQUEST] PULSE DURATION : %d\n", duration_value);

	serial_port->requestFrequencyChannel(frequency_channel);
	printf("[PACKET REQUEST] FREQUENCY CH.%d\n", frequency_channel);
}

void loopCygParser()
{
	number_of_data = serial_port->getPacketLength(packet_structure, SCAN_MAX_SIZE);

	for (uint16_t i = 0; i < number_of_data; i++)
	{
		parser_return = CygParser(total_packet_data, packet_structure[i]);

		if (parser_return == CHECKSUM_PASSED)
		{
			converData();
		}
	}
}

void converData()
{
	cyg_driver::TransformPayload TransformPayload;

	if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_2D)
	{
		TransformPayload.getDistanceArray2D(&total_packet_data[PAYLOAD_DATA], distance_buffer_2d);
		printf("BUFFER 2D Length : %ld\n", sizeof(distance_buffer_2d) / sizeof(uint16_t));
	}
	else if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_3D)
	{
		TransformPayload.getDistanceArray3D(&total_packet_data[PAYLOAD_DATA], distance_buffer_3d);
		printf("BUFFER 3D Length : %ld\n", sizeof(distance_buffer_3d) / sizeof(uint16_t));
	}
	else if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_DEV_INFO)
	{
		printf("[F/W VERSION] %d.%d.%d\n", total_packet_data[6], total_packet_data[7], total_packet_data[8]);
		printf("[H/W VERSION] %d.%d.%d\n", total_packet_data[9], total_packet_data[10], total_packet_data[11]);
	}
}
