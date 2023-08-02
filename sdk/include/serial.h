#ifndef __SERIAL_H
#define __SERIAL_H

#include <boost/asio.hpp>
#include <string>
#include <vector>

#include "cygparser.h"
#include "d_series_constant.h"

#define SCAN_MAX_SIZE 20000
#define PULSE_AUTO    0
#define PULSE_MANUAL  1

enum eRunMode
{
    Mode2D,
    Mode3D,
    ModeDual
};

class cyglidar_serial
{
    public:
        /**
            * @brief Construct CygLiDAR attached to the given serial port
            * @param port_ The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
            * @param baud_rate_ The baud rate to open the serial port at
            * @param io_ Boost ASIO IO Service to use when creating the serial port object
            */
        cyglidar_serial(const std::string &_port, uint32_t _baudrate, boost::asio::io_service &_io_service);

        /**
            * @brief Default destructor
            */
        ~cyglidar_serial()
        {
            serial.close();
        }

        /**
            * @brief Poll the laser to get a new scan. Block until a complete new scan is received or close is called.
            */
        uint16_t getPacketLength(uint8_t* _received_buffer, const uint16_t _buffer_size);

        /**
            * @brief Send a packet to run CygLiDAR
            */
        char* requestRunMode(const eRunMode _run_mode);

        /**
            * @brief Send a packet to change a width of the pulse
            */
        void requestDurationControl(const eRunMode _run_mode, uint8_t _duration_mode, uint16_t _duration_value);

        /**
            * @brief Send a packet to assign a frequency level
            */
        void requestFrequencyChannel(const uint8_t _channel_number);

        /**
            * @brief Send a packet to get a device information
            */
        void requestDeviceInfo();

        /**
            * @brief Send a packet to change baud rate setting
            */
        void requestSerialBaudRate(const uint8_t _select_baud_rate);

        /**
            * @brief Close the driver down and prevent the polling loop from advancing
            */
        void close();

    private:
        void sendCommand(const std::vector<uint8_t> &payload);

        boost::asio::serial_port serial; ///< @brief Actual serial port object for reading/writing to the lidar Scanner
        boost::system::error_code error_code;

        uint8_t payload_length_LSB;
        uint8_t payload_length_MSB;
        uint16_t number_of_packet;

        std::vector<uint8_t> command_buffer;
        std::vector<uint8_t> payload_buffer;

        char *notice;
};
#endif
