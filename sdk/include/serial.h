#ifndef __SERIAL_H
#define __SERIAL_H

#include "cygparser.h"
#include "d_series_constant.h"

#include <boost/asio.hpp>
#include <cmath>
#include <thread>
#include <iostream>
#include <string>

#define PAYLOAD_SIZE            6

#define BASE_DEPTH_2D           10000
#define BASE_ANGLE_2D           120

#define DISTANCE_MAX_2D         10000
#define SCAN_MAX_SIZE           20000

#define RIGHT_ANGLE             90
#define HALF_ANGLE              180
#define MATH_PI                 3.14159265

#define MM2M                    0.001
#define MULTIPLY_100            100
#define FOCAL_LENGTH            40.5

#define HORIZONTAL_ANGLE        120
#define VERTICAL_ANGLE          75

#define START_COUNT             0

#define BYTESET_NUM_3D          3

#define PACKET_HEADER_0         0
#define PACKET_HEADER_1         1
#define PACKET_HEADER_2         2
#define PAYLOAD_LENGTH_LSB      3
#define PAYLOAD_LENGTH_MSB      4
#define PAYLOAD_HEADER          5
#define PAYLOAD_DATA            6

#define PULSE_LIDAR_TYPE        0
#define PULSE_CONTROL_MODE      1
#define PULSE_LIDAR_3D          0
#define PULSE_LIDAR_DUAL        1
#define PULSE_AUTO              0
#define PULSE_MANUAL            1

#define PACKET_HEADER_DEV_INFO  0x10
#define PACKET_HEADER_2D        0x01
#define PACKET_HEADER_3D        0x08

#define HEX_SIZE_ONE            4
#define HEX_SIZE_TWO            8
#define HEX_SIZE_FOUR           16

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
        cyglidar_serial(const std::string& port_, uint32_t baud_rate_, boost::asio::io_service& io_);

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
        uint16_t getPacketLength(uint8_t* output_buffer_, const int buffer_size_);

        /**
            * @brief Send a packet to run CygLiDAR
            */
        std::string requestRunMode(const eRunMode run_mode_);

        /**
            * @brief Send a packet to change a width of the pulse
            */
        void requestDurationControl(const eRunMode run_mode_, const int duration_mode_, const uint16_t duration_value_);

        /**
            * @brief Send a packet to assign a frequency level
            */
        void requestFrequencyChannel(const uint8_t channel_number_);

        /**
            * @brief Send a packet to get a device information
            */
        void requestDeviceInfo();

        /**
            * @brief Close the driver down and prevent the polling loop from advancing
            */
        void close();

    private:
        std::string port;                ///< @brief The serial port which the driver belongs to
        uint32_t baud_rate;              ///< @brief The baud rate for the serial connection
        boost::asio::serial_port serial; ///< @brief Actual serial port object for reading/writing to the lidar Scanner
        uint8_t command_buffer[20];
        uint8_t payload_buffer[10];
        void makeCommand(uint8_t* command_buffer_, uint8_t* payload_, const uint16_t payload_size_);
};
#endif
