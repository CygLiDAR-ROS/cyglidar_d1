#include <thread>
#include <future>

#include "cyglidar_serial.h"
#include "cyglidar_driver.h"

using namespace std::chrono_literals;

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

std::shared_ptr<CyglidarSerial> serial_port;
std::shared_ptr<CygDriver>      cyg_driver;

std::string port;
uint8_t     baud_rate_mode;
uint8_t     run_mode;
uint8_t     duration_mode;
uint16_t    duration_value;
uint8_t     frequency_channel;

std::thread publish_thread;
std::thread double_buffer_thread;

std::shared_future<void> future;
std::promise<void> exit_signal;
std::future_status status;

std::string mode_notice;
bool info_flag = false;

uint8_t publish_done_flag  = 0;
uint8_t publish_data_state = 0;
uint8_t double_buffer_index;
uint8_t first_total_packet_data[SCAN_MAX_SIZE];
uint8_t second_total_packet_data[SCAN_MAX_SIZE];
uint8_t packet_structure[SCAN_MAX_SIZE];
uint8_t parser_return;

uint16_t number_of_data;
uint16_t distance_buffer_2d[DATA_LENGTH_2D];
uint16_t distance_buffer_3d[DATA_LENGTH_3D];

const uint8_t MODE_2D   = 0;
const uint8_t MODE_3D   = 1;
const uint8_t MODE_DUAL = 2;

const uint8_t PUBLISH_DONE = 0;
const uint8_t PUBLISH_2D   = 1;
const uint8_t PUBLISH_3D   = 2;

const uint8_t PACKET_HEADER_2D       = 0x01;
const uint8_t PACKET_HEADER_3D       = 0x08;
const uint8_t PACKET_HEADER_DEV_INFO = 0x10;

struct received_data_buffer
{
	uint8_t* packet_data;
}received_buffer[2];

void requestPacketData()
{
	serial_port->requestDeviceInfo();
	std::this_thread::sleep_for(3s);
	// sleep for 3s, by requsting the info data.

	serial_port->requestRunMode(static_cast<eRunMode>(run_mode), mode_notice);
	printf("[PACKET REQUEST] %s\n", mode_notice.c_str());

	serial_port->requestDurationControl(static_cast<eRunMode>(run_mode), duration_mode, duration_value);
	std::this_thread::sleep_for(1s);
	printf("[PACKET REQUEST] PULSE DURATION : %d\n", duration_value);

	serial_port->requestFrequencyChannel(frequency_channel);
	printf("[PACKET REQUEST] FREQUENCY CH.%d\n", frequency_channel);
}

void connectBoostSerial()
{
	try
	{
		serial_port->openSerial(port, baud_rate_mode);

		requestPacketData();
	}
	catch(const boost::system::system_error &ex)
	{
		printf("[BOOST SERIAL ERROR] %s\n", ex.what());
	}
}

void disconnectBoostSerial()
{
	serial_port->closeSerial();
	printf("[PACKET REQUEST] STOP\n");
}

void convertInfoData(received_data_buffer &_received_buffer)
{
	if (_received_buffer.packet_data[PAYLOAD_HEADER] == PACKET_HEADER_DEV_INFO && info_flag == false)
	{
		printf("[F/W VERSION] %d.%d.%d\n", received_buffer->packet_data[6], received_buffer->packet_data[7], received_buffer->packet_data[8]);
		printf("[H/W VERSION] %d.%d.%d\n", received_buffer->packet_data[9], received_buffer->packet_data[10], received_buffer->packet_data[11]);
		info_flag = true;
	}
}

void loopCygParser()
{
	number_of_data = serial_port->getPacketLength(packet_structure, SCAN_MAX_SIZE);

	for (uint16_t i = 0; i < number_of_data; i++)
	{
		parser_return = CygParser(received_buffer[double_buffer_index].packet_data, packet_structure[i]);

		if (parser_return == CHECKSUM_PASSED)
		{
			publish_done_flag |= (1 << double_buffer_index);

			double_buffer_index++;
			double_buffer_index &= 1;

			convertInfoData(received_buffer[0]);
		}
	}
}

void convertData(received_data_buffer* _received_buffer)
{
	if (_received_buffer->packet_data[PAYLOAD_HEADER] == PACKET_HEADER_2D)
	{
		cyg_driver->getDistanceArray2D(&_received_buffer->packet_data[PAYLOAD_DATA], distance_buffer_2d);
		printf("BUFFER 2D Length : %ld\n", sizeof(distance_buffer_2d) / sizeof(uint16_t));
		publish_data_state = PUBLISH_2D;
	}
	else if (_received_buffer->packet_data[PAYLOAD_HEADER] == PACKET_HEADER_3D)
	{
		cyg_driver->getDistanceArray3D(&_received_buffer->packet_data[PAYLOAD_DATA], distance_buffer_3d);
		printf("BUFFER 3D Length : %ld\n", sizeof(distance_buffer_3d) / sizeof(uint16_t));
		publish_data_state = PUBLISH_3D;
	}
}

void processDoubleBuffer()
{
	if(publish_done_flag & 0x1)
	{
		publish_done_flag &= (~0x1);
		convertData(&received_buffer[0]);
	}
	else if(publish_done_flag & 0x2)
	{
		publish_done_flag &= (~0x2);
		convertData(&received_buffer[1]);
	}
}

void doublebufferThread()
{
	do
	{
		processDoubleBuffer();
		status = future.wait_for(0s);
	} while (status == std::future_status::timeout);
}

void runPublish()
{
	if (publish_data_state == PUBLISH_3D)
	{
		publish_data_state = PUBLISH_DONE;
	}
	else if (publish_data_state == PUBLISH_2D)
	{
		publish_data_state = PUBLISH_DONE;
	}
}

void publishThread()
{
	do
	{
		runPublish();
		status = future.wait_for(0s);
	} while (status == std::future_status::timeout);
}

int main()
{
#ifdef _WIN32
			port = "COM3";
#endif
#ifdef __linux__
			port = "/dev/ttyS3";
			signal(SIGINT, interruptSignal);
#endif

	baud_rate_mode    = 0;
	run_mode 	      = MODE_DUAL;
	duration_mode     = PULSE_AUTO;
	duration_value    = 10000;
	frequency_channel = 0;
	
	serial_port = std::make_shared<CyglidarSerial>();
	cyg_driver  = std::make_shared<CygDriver>();

	received_buffer[0].packet_data = first_total_packet_data;
	received_buffer[1].packet_data = second_total_packet_data;

	future = exit_signal.get_future();
	double_buffer_thread = std::thread(doublebufferThread);
	publish_thread = std::thread(publishThread);

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
		printf("[D1 NODE ERROR] %s\n", e.what());
	}

	exit_signal.set_value();
	
	publish_thread.join();
	double_buffer_thread.join();

	return 0;
}
