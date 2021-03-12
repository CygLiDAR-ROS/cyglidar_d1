#ifndef CYGLIDAR_H
#define CYGLIDAR_H

#include <boost/asio.hpp>
#include <string>

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
      uint8_t* poll(int version_num); 

      /**
        * @brief Send a packet
        */
      void packet(int version_num);

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
