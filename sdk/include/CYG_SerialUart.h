#pragma once

#include <boost/asio.hpp>
#include <string>
#include <iostream>
#include <vector>

#include "CYG_Constant.h"

class CYG_SerialUart
{
    public:
        CYG_SerialUart();
        virtual ~CYG_SerialUart();

        void openSerialPort(const std::string &_port, const uint8_t _baudrate);
        void closeSerialPort();

        uint16_t getPacketLength(uint8_t* _received_buffer);

        void requestRunMode(const uint8_t _run_mode, std::string &_notice);
        void requestDurationControl(const uint8_t _run_mode, const uint8_t _duration_mode, const uint16_t _duration_value);
        void requestFrequencyChannel(const uint8_t _channel_number);
        void requestDeviceInfo();
        void requestSerialBaudRate(const uint8_t _select_baud_rate);

        uint32_t getBaudRate(uint8_t _baud_rate_mode);

    private:
        void transferPacketCommand(const std::vector<uint8_t> &payload);

        std::shared_ptr<boost::asio::serial_port> serial_port;
        boost::asio::io_service   io_service;
        boost::system::error_code error_code;

        uint8_t  payload_length_LSB;
        uint8_t  payload_length_MSB;
        uint16_t number_of_packet;
        uint32_t baud_rate;

        std::vector<uint8_t> command_buffer;
        std::vector<uint8_t> payload_buffer;
};
