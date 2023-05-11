#include "serial.h"
#include "point_cloud_maker.h"
#include "d_series_constant.h"
#include "cyglidar_driver.h"
#include "d1_2d_topic.h"
#include "d1_3d_topic.h"

#include <chrono>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    std::string port;
    int baud_rate;
    std::string frame_id;
    int run_mode;
    int duration_mode;
    int duration_value;
    int frequency_channel;

    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("cyglidar_publisher");

    node->declare_parameter("port");
    node->declare_parameter("baud_rate");
    node->declare_parameter("frame_id");
    node->declare_parameter("run_mode");
    node->declare_parameter("duration_mode");
    node->declare_parameter("duration_value");
    node->declare_parameter("frequency_channel");

    node->get_parameter_or<std::string>("port", port, "/dev/ttyUSB0");
    node->get_parameter_or<int>("baud_rate", baud_rate, 3000000);
    node->get_parameter_or<std::string>("frame_id", frame_id, "laser_frame");
    node->get_parameter_or<int>("run_mode", run_mode, 2);
    node->get_parameter_or<int>("duration_mode", duration_mode, PULSE_AUTO);
    node->get_parameter_or<int>("duration_value", duration_value, 10000);
    node->get_parameter_or<int>("frequency_channel", frequency_channel, 0);

    ColorRGB colorRGB;
    D1::Topic_2D  Topic_2D;
    D1::Topic_3D  Topic_3D;
    cyg_driver::TransformPayload TransformPayload;

    // Call the following function so as to store colors to draw 3D data
    colorRGB.initColorMap();

    float param_x[CygLiDARD1::Sensor::numPixel], param_y[CygLiDARD1::Sensor::numPixel], param_z[CygLiDARD1::Sensor::numPixel];
    PointCloudMaker pointcloud_3d = PointCloudMaker(param_x, param_y, param_z, CygLiDARD1::Sensor::numPixel);

    pointcloud_3d.initLensTransform(CygLiDARD1::Sensor::PixelRealSize, CygLiDARD1::Sensor::Width, CygLiDARD1::Sensor::Height,
                                    CygLiDARD1::Parameter::OffsetCenterPoint_x, CygLiDARD1::Parameter::OffsetCenterPoint_y);

    boost::asio::io_service io;

    try
    {
        // Open the port
        cyglidar_pcl serial_port(port, baud_rate, io);

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       publisher_image     = node->create_publisher<sensor_msgs::msg::Image>      ("scan_image", 5);
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   publisher_laserscan = node->create_publisher<sensor_msgs::msg::LaserScan>  ("scan", 1000);
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_2d  = node->create_publisher<sensor_msgs::msg::PointCloud2>("scan_2D", 5);
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_3d  = node->create_publisher<sensor_msgs::msg::PointCloud2>("scan_3D", 5);

        RCLCPP_INFO(node->get_logger(), "%s", serial_port.requestRunMode(static_cast<eRunMode>(run_mode)).c_str());

        serial_port.requestDurationControl(static_cast<eRunMode>(run_mode), duration_mode, duration_value);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(node->get_logger(), "[PACKET UPDATED] PULSE DURATION : %d", duration_value);
        // sleep for a sec, by the duration

        serial_port.requestFrequencyChannel(frequency_channel);
        RCLCPP_INFO(node->get_logger(), "[PACKET UPDATED] FREQUENCY CH.%d", frequency_channel);

        sensor_msgs::msg::Image::SharedPtr      scan_image(new sensor_msgs::msg::Image);
        sensor_msgs::msg::LaserScan::SharedPtr  scan_laser(new sensor_msgs::msg::LaserScan);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_2D(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_3D(new pcl::PointCloud<pcl::PointXYZRGBA>);

        rclcpp::Time scan_start_time = node->now() - rclcpp::Duration(0, 48000*1000);
        double scan_duration = 0;

        uint8_t total_packet_data[SCAN_MAX_SIZE];
        uint8_t packet_structure[SCAN_MAX_SIZE];

        uint16_t distance_buffer_2d[cyg_driver::DATA_LENGTH_2D];
        uint16_t distance_buffer_3d[cyg_driver::DATA_LENGTH_3D];
        uint16_t number_of_data = 0;

        while (rclcpp::ok())
        {
            number_of_data = serial_port.getPacketLength(packet_structure, SCAN_MAX_SIZE);
            for (uint16_t i = 0; i < number_of_data; i++)
            {
                if(CygParser(total_packet_data, packet_structure[i]) == CHECKSUM_PASSED)
                {
                    if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_2D)
                    {
                        TransformPayload.getDistanceArray2D(&total_packet_data[PAYLOAD_DATA], distance_buffer_2d);

                        Topic_2D.publishScanLaser(publisher_laserscan, scan_laser, frame_id, scan_start_time, scan_duration, distance_buffer_2d);
                        Topic_2D.publishPoint2D(publisher_point_2d, scan_2D, frame_id, distance_buffer_2d);
                    }
                    else if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_3D)
                    {
                        TransformPayload.getDistanceArray3D(&total_packet_data[PAYLOAD_DATA], distance_buffer_3d);

                        Topic_3D.publishScanImage(publisher_image, scan_image, frame_id, distance_buffer_3d);
                        Topic_3D.publishPoint3D(publisher_point_3d, scan_3D, frame_id, pointcloud_3d, colorRGB.color_map, distance_buffer_3d);
                    }
                }
            }
            rclcpp::spin_some(node->shared_from_this());
        }
        serial_port.close();
        RCLCPP_INFO(node->get_logger(), "PACKET UPDATED : STOP");
        return 0;
    }
    catch (const boost::system::system_error& ex)
    {
        RCLCPP_ERROR(node->get_logger(), "An exception was thrown [%s]", ex.what());
        return -1;
    }
}
