#ifndef __SERIAL_H
#define __SERIAL_H

#include <boost/asio.hpp>
#include <string>
#include <iostream>
#include <vector>

#include "cygparser.h"
#include "d_series_constant.h"

#define SCAN_MAX_SIZE 20000
#define PULSE_AUTO    0
#define PULSE_MANUAL  1

using namespace Constant_D1::Command;

enum eRunMode
{
    Mode2D,
    Mode3D,
    ModeDual
};

class CyglidarSerial
{
    public:
        /**
            * @brief Default constructor
            */
        CyglidarSerial();

        /**
            * @brief Default destructor
            */
        virtual ~CyglidarSerial();

        /**
            * @brief Construct CygLiDAR attached to the given serial port
            * @param _port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
            * @param _baudrate The baud rate to open the serial port at
            */
        void openSerial(const std::string &_port, const uint8_t _baudrate);

        /**
            * @brief Poll the laser to get a new scan. Block until a complete new scan is received or close is called.
            */
        uint16_t getPacketLength(uint8_t* _received_buffer, const uint16_t _buffer_size);

        /**
            * @brief Send a packet to run CygLiDAR
            */
        void requestRunMode(const eRunMode _run_mode, std::string &_notice);

        /**
            * @brief Send a packet to change a width of the pulse
            */
        void requestDurationControl(const eRunMode _run_mode, const uint8_t _duration_mode, const uint16_t _duration_value);

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
            * @brief Get a value to set baud rate
            */
        uint32_t getBaudRate(uint8_t _baud_rate_mode);

        /**
            * @brief Close the driver down and prevent the polling loop from advancing
            */
        void closeSerial();

    private:
        void sendCommand(const std::vector<uint8_t> &payload);

        std::shared_ptr<boost::asio::serial_port> serial; ///< @brief Actual serial port object for reading/writing to the lidar Scanner
        boost::asio::io_service io_service;
        boost::system::error_code error_code;

        uint8_t  payload_length_LSB;
        uint8_t  payload_length_MSB;
        uint16_t number_of_packet;
        uint32_t baud_rate;

        std::vector<uint8_t> command_buffer;
        std::vector<uint8_t> payload_buffer;
};
#endif
