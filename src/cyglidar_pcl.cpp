#include <cyglidar_pcl.h>
#include <iostream>
#include <ros/ros.h>

#define START_COUNT            0
#define DATABUFFER_SIZE_2D     249
#define DATABUFFER_SIZE_3D     14407

static boost::array<uint8_t, 8> PACKET_START_2D = { 0x5A, 0x77, 0xFF, 0x02, 0x00, 0x01, 0x00, 0x03 };
static boost::array<uint8_t, 8> PACKET_START_3D = { 0x5A, 0x77, 0xFF, 0x02, 0x00, 0x08, 0x00, 0x0A };
static boost::array<uint8_t, 8> PACKET_START_DUAL = { 0x5A, 0x77, 0xFF, 0x02, 0x00, 0x07, 0x00, 0x05 };
static boost::array<uint8_t, 8> PACKET_STOP = { 0x5A, 0x77, 0xFF, 0x02, 0x00, 0x02, 0x00, 0x00 };

static boost::array<uint8_t, 9> PACKET_INTEGRATION_TIME = { 0x5A, 0x77, 0xFF, 0x03, 0x00, 0x0C, 0x00, 0x00, 0x00 };

//uint8_t raw_bytes[DATABUFFER_SIZE_3D + 2]; // *raw_bytes = new uint8_t[DATABUFFER_SIZE_3D + 2];

using namespace std;
namespace cyglidar_pcl_driver 
{
    cyglidar_pcl::cyglidar_pcl(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
    :port_(port), baud_rate_(baud_rate), serial_(io, port_)
    {
        try
        {
            serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("[Baud Rate] %d / %d\nCaught Exception: %s",
            baud_rate, baud_rate_, e.what());
            return;
        }
    }

    cyglidar_pcl::~cyglidar_pcl()
    {
        serial_.close();
    }

	boost::system::error_code errorCode;
	int dataCnt = 0;

	//uint8_t *raw_bytes = new uint8_t[DATABUFFER_SIZE_3D + 2];
	boost::array<uint8_t, DATABUFFER_SIZE_3D + 2> raw_bytes;

    uint8_t* cyglidar_pcl::poll(int version_num)
    {
        //dataCnt = serial_.read_some(boost::asio::buffer(raw_bytes, DATABUFFER_SIZE_3D + 2), errorCode);
        dataCnt = boost::asio::read(serial_, boost::asio::buffer(raw_bytes), boost::asio::transfer_at_least(1), errorCode);

		if (errorCode)
		{
			raw_bytes[DATABUFFER_SIZE_3D] = 0x00; // MSB
			raw_bytes[DATABUFFER_SIZE_3D + 1] = 0x00; // LSB
		}
		else
		{
			raw_bytes[DATABUFFER_SIZE_3D] = (dataCnt) >> 8; // MSB
			raw_bytes[DATABUFFER_SIZE_3D + 1] = (dataCnt) & 0x00ff; // LSB
		}

		return &raw_bytes[0];
/*
		boost::asio::read(serial_, boost::asio::buffer(&raw_data[0], 1));
		return &raw_data[0];*/

		//ROS_INFO("DATA COUNT: %d => %x, %x", dataCnt, raw_bytes[DATABUFFER_SIZE_3D], raw_bytes[DATABUFFER_SIZE_3D + 1]);
    }

    void cyglidar_pcl::packet(int version_num)
    {
        switch (version_num)
        {
            case 0: // 2D
                boost::asio::write(serial_, boost::asio::buffer(PACKET_START_2D));
                break;
            case 1: // 3D
                boost::asio::write(serial_, boost::asio::buffer(PACKET_START_3D));
                break;
            case 2: // Dual
                boost::asio::write(serial_, boost::asio::buffer(PACKET_START_DUAL));
                break;
        }
	    ROS_INFO("PACKET SENT (START, %d)", version_num);

		// 3D Pulse Duration Control (Integration Time)
		if (version_num != 0)
		{
			if (version_num == 1)
			{
				PACKET_INTEGRATION_TIME[6] = 10001000;
				PACKET_INTEGRATION_TIME[7] = 01010011;
			}
			else 
			{
				PACKET_INTEGRATION_TIME[6] = 10001000;
				PACKET_INTEGRATION_TIME[7] = 11010011;
			}

			uint8_t checkSum = 0x00;
			for (size_t t = 3; t < PACKET_INTEGRATION_TIME.size() - 1; t++)
			{
				checkSum ^= PACKET_INTEGRATION_TIME[t];
			}
			PACKET_INTEGRATION_TIME[PACKET_INTEGRATION_TIME.size() - 1] = checkSum;
		    boost::asio::write(serial_, boost::asio::buffer(PACKET_INTEGRATION_TIME));
			ROS_INFO("PACKET_INTEGRATION_TIME WAS JUST FIRED... (%x, %x)", PACKET_INTEGRATION_TIME[5], PACKET_INTEGRATION_TIME[6]);
		}
    }

    void cyglidar_pcl::close()
    {
        boost::asio::write(serial_, boost::asio::buffer(PACKET_STOP));
        ROS_INFO("PACKET SENT (STOP)");
    }

}