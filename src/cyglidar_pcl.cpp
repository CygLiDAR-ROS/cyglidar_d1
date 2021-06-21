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
            ROS_ERROR("[Baud Rate] %d / %d\nCaught Exception: %s",
            baud_rate, baud_rate_, e.what());
            return;
        }
    }

    cyglidar_pcl::~cyglidar_pcl()
    {
        serial_.close();
    }

    uint8_t IntToHex(int value)
    {
        switch (value)
        {
            case 0:
                currentHex = 0x00;
                break;
            case 1:
                currentHex = 0x01;
                break;
            case 2:
                currentHex = 0x02;
                break;
            case 3:
                currentHex = 0x03;
                break;
            case 4:
                currentHex = 0x04;
                break;
            case 5:
                currentHex = 0x05;
                break;
            case 6:
                currentHex = 0x06;
                break;
            case 7:
                currentHex = 0x07;
                break;
            case 8:
                currentHex = 0x08;
                break;
            case 9:
                currentHex = 0x09;
                break;
            case 10:
                currentHex = 0x0A;
                break;
            case 11:
                currentHex = 0x0B;
                break;
            case 12:
                currentHex = 0x0C;
                break;
            case 13:
                currentHex = 0x0D;
                break;
            case 14:
                currentHex = 0x0E;
                break;
            case 15:
                currentHex = 0x0F;
                break;
        }
        return currentHex;
    }
    
    uint32_t AccumHex(uint8_t hex, int hexLoop)
    {
        completedHex = (completedHex | (hex << (hexLoop * HEX_SIZE_ONE)));
        return completedHex;
    }

    uint32_t DecimalToHex(int value)
    {
        completedHex = 0, hexLoop = 0;
        int modFreq = value;
        //printf("DecimalToHex ==> %d, %d\t==>\t%d(%x),", value, modFreq, modFreq % 16, IntToHex(modFreq % 16));
        tempHex = IntToHex(modFreq % 16);
        AccumHex(tempHex, hexLoop++);
        while (modFreq != 0)
        {
            modFreq = ((modFreq - (modFreq % 16)) / 16);
            if (modFreq > 0)
            {
                tempHex = IntToHex(modFreq % 16);
                AccumHex(tempHex, hexLoop++);
                //printf("%d(%x),", modFreq % 16, tempHex);
            }
        }
        //ROS_INFO("DecimalToHex: %x(%d)", completedHex, completedHex);
        return completedHex;
    }
    
    void DecimalToBinary(int value)
    {
        binaryBuf_size = (BINARY_BUFFER.size() / sizeof(char));
        binary_index = (binaryBuf_size - 1);

        int modFreq = value;
        //printf("DecimalToBinary ==> %d, %d ==> %d,", value, modFreq, modFreq % 2);
        BINARY_BUFFER[binary_index--] = (modFreq % 2);
        while (modFreq != 0)
        {
            modFreq = ((modFreq - (modFreq % 2)) / 2);
            if (modFreq > 0)
            {
                BINARY_BUFFER[binary_index--] = (modFreq % 2);
            }
        }
        //printf("binary_index: %d / BINARY_BUFFER.size(): %d\t==>\t", binaryBuf_size - binary_index, binaryBuf_size);
        
        if (binary_index > 0)
        {
            for (int b = binary_index; b > -1; b--)
            {
                BINARY_BUFFER[b] = 0;
            }
        }/*

        printf("DecimalToBinary: ");
        for (int bSize = 0; bSize < binaryBuf_size; bSize++)
        {
            printf("%d", BINARY_BUFFER[bSize]);
            if (bSize % 4 == 3) printf(" ");
        }
        printf("\n");*/
    }

	boost::system::error_code errorCode;
	int dataCnt = 0;

	//uint8_t *raw_bytes = new uint8_t[SIZE_MAX + 2];
	boost::array<uint8_t, SIZE_MAX + 2> raw_bytes;

    uint8_t* cyglidar_pcl::poll(int version)
    {
        //dataCnt = serial_.read_some(boost::asio::buffer(raw_bytes, SIZE_MAX + 2), errorCode);
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
/*
		boost::asio::read(serial_, boost::asio::buffer(&raw_data[0], 1));
		return &raw_data[0];*/

		//ROS_INFO("DATA COUNT: %d => %x, %x", dataCnt, raw_bytes[SIZE_MAX], raw_bytes[SIZE_MAX + 1]);
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
                DecimalToBinary(duration);
                //printf("Binary Splitting\t=>\t");
                for (int bSize = 0; bSize < binaryBuf_size; bSize++)
                {
                    if (bSize > (HEX_SIZE_TWO - 1))
                    {
                        LSB_BUFFER[bSize - HEX_SIZE_TWO] = BINARY_BUFFER[bSize];
                        //printf("%d", LSB_BUFFER[bSize - HEX_SIZE_TWO]);
                    }
                    else
                    {
                        MSB_BUFFER[bSize] = BINARY_BUFFER[bSize];
                        //printf("%d", MSB_BUFFER[bSize]);
                        //if (bSize == (HEX_SIZE_TWO - 1)) printf(" ");
                    }
                }//printf("\n");
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


            //printf("Binary Splitting\t=>\t");
            for (int bSize = 0; bSize < binaryBuf_size; bSize++)
            {
                if (bSize > (HEX_SIZE_TWO - 1))
                {
                    //printf("%d", LSB_BUFFER[bSize - HEX_SIZE_TWO]);
                    if (LSB_BUFFER[bSize - HEX_SIZE_TWO] > 0)
                    {
                        LSB_int += pow(2, HEX_SIZE_TWO - (bSize - HEX_SIZE_TWO) - 1);
                    }
                }
                else
                {
                    //printf("%d", MSB_BUFFER[bSize]);
                    //if (bSize == (HEX_SIZE_TWO - 1)) printf(" ");
                    if (MSB_BUFFER[bSize] > 0)
                    {
                        MSB_int += pow(2, (HEX_SIZE_TWO - bSize - 1));
                    }
                }
            }
            //printf("\t==>\t%d, %d\n", MSB_int, LSB_int);

            PACKET_INTEGRATION_TIME[6] = DecimalToHex(LSB_int);
            PACKET_INTEGRATION_TIME[7] = DecimalToHex(MSB_int);

			checkSum = 0x00;
			for (size_t t = 3; t < PACKET_INTEGRATION_TIME.size() - 1; t++)
			{
				checkSum ^= PACKET_INTEGRATION_TIME[t];
			}
            PACKET_INTEGRATION_TIME[PACKET_INTEGRATION_TIME.size() - 1] = checkSum;

		    boost::asio::write(serial_, boost::asio::buffer(PACKET_INTEGRATION_TIME));
			ROS_INFO("PACKET_INTEGRATION_TIME HAS BEEN APPLIED [%d]", duration);

            printf("\tPACKET: ");
            for (size_t buf = 0; buf < PACKET_INTEGRATION_TIME.size(); buf++)
            {
                printf("%x", PACKET_INTEGRATION_TIME[buf]);
                if (buf < (PACKET_INTEGRATION_TIME.size() - 1)) printf(",");
            }
            printf("\n");
        }
    }

    void cyglidar_pcl::packet_frequency(int frequency)
    {
        PACKET_FREQUENCY[6] = DecimalToHex(frequency);
            
        checkSum = 0x00;
        for (size_t t = 3; t < PACKET_FREQUENCY.size() - 1; t++)
        {
            checkSum ^= PACKET_FREQUENCY[t];
        }
        PACKET_FREQUENCY[PACKET_FREQUENCY.size() - 1] = checkSum;

        boost::asio::write(serial_, boost::asio::buffer(PACKET_FREQUENCY));
        ROS_INFO("PACKET_FREQUENCY HAS BEEN UPDATED [%d]", frequency);

        printf("\tPACKET: ");
        for (size_t buf = 0; buf < PACKET_FREQUENCY.size(); buf++)
        {
            printf("%x", PACKET_FREQUENCY[buf]);
            if (buf < (PACKET_FREQUENCY.size() - 1)) printf(",");
        }
        printf("\n");
    }

    void cyglidar_pcl::close()
    {
        boost::asio::write(serial_, boost::asio::buffer(PACKET_STOP));
        ROS_INFO("PACKET SENT (STOP)");
    }

}