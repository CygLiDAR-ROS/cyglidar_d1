#ifndef CYGLIDAR_H
#define CYGLIDAR_H

#include <ros/ros.h>
#include <CygbotParser.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>
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

#define PACKET_HEADER_0         0x5A
#define PACKET_HEADER_1         0x77
#define PACKET_HEADER_2         0xFF

#define PULSE_LIDAR_TYPE        0
#define PULSE_CONTROL_MODE      1
#define PULSE_LIDAR_3D          0
#define PULSE_LIDAR_DUAL        1
#define PULSE_AUTO              0
#define PULSE_MANUAL            1

#define HEX_SIZE_ONE            4
#define HEX_SIZE_TWO            8
#define HEX_SIZE_FOUR           16

enum eRunMode
{
  Mode2D,
  Mode3D,
  ModeDual
};

class cyglidar_pcl
{
  public:

      /**
        * @brief Construct CygLiDAR attached to the given serial port
        * @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
        * @param baud_rate The baud rate to open the serial port at
        * @param io Boost ASIO IO Service to use when creating the serial port object
        */
      cyglidar_pcl(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io);

      /**
        * @brief Default destructor
        */
      ~cyglidar_pcl()
      {
          serial_.close();
      }

      /**
        * @brief Poll the laser to get a new scan. Block until a complete new scan is received or close is called.
        */
       uint16_t rvData(uint8_t* outputBuffer, const int BufferSize);

      /**
        * @brief Send a packet to run CygLiDAR
        */
      void packet_run(const eRunMode mode);

      /**
        * @brief Send a packet to change a width of the pulse
        */
      void packet_duration(const eRunMode mode, const bool setAutoDuration, const uint16_t Duration);

      /**
        * @brief Send a packet to assign a frequency level
        */
       void packet_frequency(const uint8_t channel);

      /**
        * @brief Print every element of the array after sending the packet
        */
      void packet_confirmation(uint8_t* buffer, int count);

      /**
        * @brief Close the driver down and prevent the polling loop from advancing
        */
      void close();
  private:
      uint8_t CommandBuffer[20];
      uint8_t payloadBuffer[10];
      void makeCommand(uint8_t* commandBuffer, uint8_t* Payload, const uint16_t payloadSize);
      std::string port_; ///< @brief The serial port which the driver belongs to
      uint32_t baud_rate_; ///< @brief The baud rate for the serial connection
      boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the lidar Scanner
};

#endif // CYGLIDAR_H
