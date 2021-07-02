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

#define BASE_DEPTH_3D           3000
#define BASE_DEPTH_2D           10000
#define BASE_ANGLE_2D           120

#define DISTANCE_MAX_2D         10000
#define SIZE_MAX                20000

#define INVALID_DATA_2D         16000
#define LOW_AMPLITUDE_2D        16001
#define ADC_OVERFLOW_2D         16002
#define SATURATION_VALUE_2D     16003
#define BAD_PIXEL_2D            16004

#define TRACKING_VALUE_3D       4001
#define INVALID_DATA_3D         4080
#define LOW_AMPLITUDE_3D        4081
#define ADC_OVERFLOW_3D         4082
#define SATURATION_VALUE_3D     4083
#define INTERFERENCE_VALUE_3D   4087

#define ADC_COL_R               173
#define ADC_COL_G               216
#define ADC_COL_B               230

#define SAT_COL_R               128
#define SAT_COL_G               0
#define SAT_COL_B               128

#define COLOR_MIN               0
#define COLOR_MAX               255

#define RIGHT_ANGLE             90
#define HALF_ANGLE              180
#define MATH_PI                 3.14159265

#define DIVISOR                 0.001
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

static boost::array<uint8_t, 8> PACKET_START_2D = { PACKET_HEADER_0, PACKET_HEADER_1, PACKET_HEADER_2, 0x02, 0x00, 0x01, 0x00, 0x03 };
static boost::array<uint8_t, 8> PACKET_START_3D = { PACKET_HEADER_0, PACKET_HEADER_1, PACKET_HEADER_2, 0x02, 0x00, 0x08, 0x00, 0x0A };
static boost::array<uint8_t, 8> PACKET_START_DUAL = { PACKET_HEADER_0, PACKET_HEADER_1, PACKET_HEADER_2, 0x02, 0x00, 0x07, 0x00, 0x05 };
static boost::array<uint8_t, 8> PACKET_STOP = { PACKET_HEADER_0, PACKET_HEADER_1, PACKET_HEADER_2, 0x02, 0x00, 0x02, 0x00, 0x00 };

static boost::array<uint8_t, 8> PACKET_FREQUENCY = { PACKET_HEADER_0, PACKET_HEADER_1, PACKET_HEADER_2, 0x02, 0x00, 0x0F, 0x00, 0x00 };
static boost::array<uint8_t, 9> PACKET_INTEGRATION_TIME = { PACKET_HEADER_0, PACKET_HEADER_1, PACKET_HEADER_2, 0x03, 0x00, 0x0C, 0x00, 0x00, 0x00 };

static boost::array<char, HEX_SIZE_TWO> MSB_BUFFER, LSB_BUFFER;
static boost::array<char, HEX_SIZE_FOUR> BINARY_BUFFER;

namespace cyglidar_pcl_driver {
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
      ~cyglidar_pcl();

      /**
        * @brief Poll the laser to get a new scan. Block until a complete new scan is received or close is called.
        * @param scan 
        */
      uint8_t* poll(int version); 

      /**
        * @brief Send a packet to run CygLiDAR
        */
      void packet_run(int version);

      /**
        * @brief Send a packet to change a width of the pulse
        */
      void packet_pulse(int version, int pulse_control, int duration);

      /**
        * @brief Send a packet to assign a frequency level
        */
      void packet_frequency(int frequency);

      /**
        * @brief Close the driver down and prevent the polling loop from advancing
        */
      void close();
  private:
      std::string port_; ///< @brief The serial port which the driver belongs to
      uint32_t baud_rate_; ///< @brief The baud rate for the serial connection
      boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the lidar Scanner
  };
}

#endif // CYGLIDAR_H
