#ifndef CYGLIDAR_H
#define CYGLIDAR_H

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>

namespace cyglidar_pcl_driver {
class cyglidar_pcl
{
  public:

      /**
        * @brief Constructs a new CygLiDAR attached to the given serial port
        * @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
        * @param baud_rate The baud rate to open the serial port at.
        * @param io Boost ASIO IO Service to use when creating the serial port object
        */
      cyglidar_pcl(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io);

      /**
        * @brief Default destructor
        */
      ~cyglidar_pcl();

      /**
        * @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
        * @param scan 
        */
      uint8_t* poll(int version_num); 

      /**
        * @brief Send packets
        */
      void packet(int version_num);

      /**
        * @brief Close the driver down and prevent the polling loop from advancing
        */
      void close();
  private:
      std::string port_; ///< @brief The serial port the driver is attached to
      uint32_t baud_rate_; ///< @brief The baud rate for the serial connection

      bool shutting_down_; ///< @brief Flag for whether the driver is supposed to be shutting down or not
      boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the n301n lidar Scanner
  };
}

#endif // CYGLIDAR_H
