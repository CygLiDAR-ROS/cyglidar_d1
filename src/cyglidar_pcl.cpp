#include <cyglidar_pcl.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <CygbotParser.h>

#define START_COUNT            0
#define DATABUFFER_SIZE_2D     249
#define DATABUFFER_SIZE_3D     14407

static boost::array<uint8_t, 8> PACKET_START_2D = { 0x5A, 0x77, 0xFF, 0x02, 0x00, 0x01, 0x00, 0x03 };
static boost::array<uint8_t, 8> PACKET_START_3D = { 0x5A, 0x77, 0xFF, 0x02, 0x00, 0x08, 0x00, 0x0A };
static boost::array<uint8_t, 8> PACKET_START_DUAL = { 0x5A, 0x77, 0xFF, 0x02, 0x00, 0x07, 0x00, 0x05 };
static boost::array<uint8_t, 8> PACKET_STOP = { 0x5A, 0x77, 0xFF, 0x02, 0x00, 0x02, 0x00, 0x00 };

static boost::array<uint8_t, 8> PACKET_INTEGRATION_ON = { 0x5A, 0x77, 0xFF, 0x02, 0x00, 0x0D, 0x01, 0x0E };
static boost::array<uint8_t, 8> PACKET_INTEGRATION_OFF = { 0x5A, 0x77, 0xFF, 0x02, 0x00, 0x0D, 0x00, 0x0F };
static boost::array<uint8_t, 8> PACKET_INTEGRATION_TIME = { 0x5A, 0x77, 0xFF, 0x03, 0x00, 0x0C, 0x00, 0x00 };

using namespace std;
namespace cyglidar_pcl_driver 
{
    cyglidar_pcl::cyglidar_pcl(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
    :port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
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

    uint8_t result = 0;
    uint8_t *originalBuffer = new uint8_t[DATABUFFER_SIZE_3D];
    uint8_t *originalBuffer_01 = new uint8_t[DATABUFFER_SIZE_3D];
    uint8_t *originalBuffer_02 = new uint8_t[DATABUFFER_SIZE_3D];
    boost::array<uint8_t, 1> raw_bytes;

    uint8_t* cyglidar_pcl::poll(int version_num)
    {
        while (!shutting_down_)
        {
            boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[START_COUNT], 1));

            result = CygParser(originalBuffer, raw_bytes[START_COUNT]);
            if (result == 0x01)
            {
                return originalBuffer;
            }
        }

        return false;
    }

    void cyglidar_pcl::packet(int version_num)
    {
        switch (version_num)
        {
            case 0: // 2D
                boost::asio::write(serial_, boost::asio::buffer(PACKET_START_2D));
                break;
            case 1: // 3D
                boost::asio::write(serial_, boost::asio::buffer(PACKET_START_3D));
                boost::asio::write(serial_, boost::asio::buffer(PACKET_INTEGRATION_ON));
                break;
            case 2: // Dual
                boost::asio::write(serial_, boost::asio::buffer(PACKET_START_DUAL));
                break;
        }
        
        ROS_INFO("PACKET SENT (START, %d)", version_num);
    }

    void cyglidar_pcl::close()
    {
        boost::asio::write(serial_, boost::asio::buffer(PACKET_STOP));
        ROS_INFO("PACKET SENT (STOP)");
        
        shutting_down_ = true;
    }

}

