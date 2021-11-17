#include <cyglidar_pcl.h>

uint8_t checkSum;
uint8_t currentHex;
uint8_t MSB_hex, LSB_hex;
uint32_t completedHex;
int tempHex, hexLoop;
int binary_index, binaryBuf_size;
int MSB_int, LSB_int;

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
            ROS_ERROR("Error Caught: %s", e.what());
            return;
        }
    }

    cyglidar_pcl::~cyglidar_pcl()
    {
        serial_.close();
    }

    uint32_t AccumHex(uint8_t hex, int hexLoop)
    {
        completedHex = (completedHex | (hex << (hexLoop * HEX_SIZE_ONE)));
        return completedHex;
    }

	boost::system::error_code errorCode;
	int dataCnt = 0;

	boost::array<uint8_t, SIZE_MAX + 2> raw_bytes;

    uint8_t* cyglidar_pcl::poll(int version)
    {
        dataCnt = boost::asio::read(serial_, boost::asio::buffer(raw_bytes), boost::asio::transfer_at_least(1), errorCode);

		if (errorCode)
		{
			raw_bytes[SIZE_MAX] = 0x00; // MSB
			raw_bytes[SIZE_MAX + 1] = 0x00; // LSB
		}
		else
		{
			raw_bytes[SIZE_MAX] = (dataCnt) >> 8; // MSB
			raw_bytes[SIZE_MAX + 1] = (dataCnt) & 0x00ff; // LSB
		}

		return &raw_bytes[0];
    }

    void cyglidar_pcl::packet_run(int version)
    {
        switch (version)
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
	    ROS_INFO("PACKET SENT (START, %d)", version);
    }

    void cyglidar_pcl::packet_pulse(int version, int pulse_control, int duration)
    {
        if (version > 0)
        {
            MSB_int = 0, LSB_int = 0;
            if (pulse_control == 1)
            {
                for (int bSize = 0; bSize < binaryBuf_size; bSize++)
                {
                    if (bSize > (HEX_SIZE_TWO - 1))
                    {
                        LSB_BUFFER[bSize - HEX_SIZE_TWO] = BINARY_BUFFER[bSize];
                    }
                    else
                    {
                        MSB_BUFFER[bSize] = BINARY_BUFFER[bSize];
                    }
                }
                MSB_BUFFER[PULSE_CONTROL_MODE] = PULSE_MANUAL;
            }
            else
            {
                MSB_BUFFER[PULSE_CONTROL_MODE] = PULSE_AUTO;
            }

            switch (version)
            {
                case 1:
                    MSB_BUFFER[PULSE_LIDAR_TYPE] = PULSE_LIDAR_3D;
                    break;
                case 2:
                    MSB_BUFFER[PULSE_LIDAR_TYPE] = PULSE_LIDAR_DUAL;
                    break;
            }

            for (int bSize = 0; bSize < binaryBuf_size; bSize++)
            {
                if (bSize > (HEX_SIZE_TWO - 1))
                {
                    if (LSB_BUFFER[bSize - HEX_SIZE_TWO] > 0)
                    {
                        LSB_int += pow(2, HEX_SIZE_TWO - (bSize - HEX_SIZE_TWO) - 1);
                    }
                }
                else
                {
                    if (MSB_BUFFER[bSize] > 0)
                    {
                        MSB_int += pow(2, (HEX_SIZE_TWO - bSize - 1));
                    }
                }
            }

            PACKET_INTEGRATION_TIME[6] = LSB_int;
            PACKET_INTEGRATION_TIME[7] = MSB_int;

			checkSum = 0x00;
			for (size_t t = 3; t < PACKET_INTEGRATION_TIME.size() - 1; t++)
			{
				checkSum ^= PACKET_INTEGRATION_TIME[t];
			}
            PACKET_INTEGRATION_TIME[PACKET_INTEGRATION_TIME.size() - 1] = checkSum;

		    boost::asio::write(serial_, boost::asio::buffer(PACKET_INTEGRATION_TIME));
			ROS_INFO("PACKET_INTEGRATION_TIME HAS BEEN APPLIED [%d]", duration);
            packet_confirmation(&PACKET_INTEGRATION_TIME[0], PACKET_INTEGRATION_TIME.size());
        }
    }

    void cyglidar_pcl::packet_frequency(int frequency)
    {
        PACKET_FREQUENCY[6] = frequency;
            
        checkSum = 0x00;
        for (size_t t = 3; t < PACKET_FREQUENCY.size() - 1; t++)
        {
            checkSum ^= PACKET_FREQUENCY[t];
        }
        PACKET_FREQUENCY[PACKET_FREQUENCY.size() - 1] = checkSum;

        boost::asio::write(serial_, boost::asio::buffer(PACKET_FREQUENCY));
        ROS_INFO("PACKET_FREQUENCY HAS BEEN UPDATED [%d]", frequency);
        packet_confirmation(&PACKET_FREQUENCY[0], PACKET_FREQUENCY.size());
    }

    void cyglidar_pcl::packet_confirmation(uint8_t* buffer, int count)
    {
        printf("\tPACKET: ");
        for (size_t buf = 0; buf < count; buf++)
        {
            printf("%x", buffer[buf]);
            if (buf < (count - 1)) printf(",");
        }
        printf("\n");
    }

    void cyglidar_pcl::close()
    {
        boost::asio::write(serial_, boost::asio::buffer(PACKET_STOP));
        ROS_INFO("PACKET SENT (STOP)");
    }

}