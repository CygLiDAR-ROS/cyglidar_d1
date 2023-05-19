#include "d1_node.h"

namespace D1
{
    using namespace std::chrono_literals;
    using namespace CygLiDARD1;

    CygLiDAR_D1::CygLiDAR_D1(std::string name_) : Node(name_)
    {
        publisher_laserscan = this->create_publisher<sensor_msgs::msg::LaserScan>  ("scan",       1000);
        publisher_point_2d  = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_2D",    5);
        publisher_point_3d  = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_3D",    5);
        publisher_image     = this->create_publisher<sensor_msgs::msg::Image>      ("scan_image", 5);

        initParam();
        onInit();
    }

    CygLiDAR_D1::~CygLiDAR_D1(){}

    float param_x[Sensor::numPixel], param_y[Sensor::numPixel], param_z[Sensor::numPixel];
    PointCloudMaker pointcloud_maker(param_x, param_y, param_z, Sensor::numPixel);

    void CygLiDAR_D1::initParam()
    {
        this->declare_parameter("port");
        this->declare_parameter("baud_rate");
        this->declare_parameter("frame_id");
        this->declare_parameter("run_mode");
        this->declare_parameter("duration_mode");
        this->declare_parameter("duration_value");
        this->declare_parameter("frequency_channel");

        this->get_parameter_or<std::string>("port", port, "/dev/ttyUSB0");
        this->get_parameter_or<int>("baud_rate", baud_rate, 3000000);
        this->get_parameter_or<std::string>("frame_id", frame_id, "laser_frame");
        this->get_parameter_or<int>("run_mode", run_mode, 2);
        this->get_parameter_or<int>("duration_mode", duration_mode, PULSE_AUTO);
        this->get_parameter_or<int>("duration_value", duration_value, 10000);
        this->get_parameter_or<int>("frequency_channel", frequency_channel, 0);

        // Call the following function so as to store colors to draw 3D data
        pointcloud_maker.initColorMap();
        pointcloud_maker.initLensTransform(Sensor::PixelRealSize, Sensor::Width, Sensor::Height,
                                           Parameter::OffsetCenterPoint_x, Parameter::OffsetCenterPoint_y);
    }

    void CygLiDAR_D1::onInit()
    {
        boost::asio::io_service io;

        try
        {
            // Open the port
            cyglidar_serial serial_port(port, baud_rate, io);

            sendPacketData(serial_port);
            publishLoop(serial_port);
        }
        catch (const boost::system::system_error& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "An exception was thrown [%s]", ex.what());
        }
    }

    void CygLiDAR_D1::publishLoop(cyglidar_serial &serial_port_)
    {
        while (rclcpp::ok())
        {
            number_of_data = serial_port_.getPacketLength(packet_structure, SCAN_MAX_SIZE);

            for (uint16_t i = 0; i < number_of_data; i++)
            {
                if(CygParser(total_packet_data, packet_structure[i]) == CHECKSUM_PASSED)
                {
                    publishData(total_packet_data);
                }
            }
        }
        serial_port_.close();
        RCLCPP_INFO(this->get_logger(), "PACKET REQUEST : STOP");
    }

    void CygLiDAR_D1::publishData(uint8_t *total_packet_data_)
    {
        Topic_2D Topic_data_2d;
        Topic_3D Topic_data_3d;
        cyg_driver::TransformPayload TransformPayload;

        rclcpp::Time scan_start_time = this->now() - rclcpp::Duration(0, 48000*1000);

        if (total_packet_data_[PAYLOAD_HEADER] == PACKET_HEADER_2D)
        {
            TransformPayload.getDistanceArray2D(&total_packet_data_[PAYLOAD_DATA], distance_buffer_2d);

            Topic_data_2d.publishPoint2D(frame_id, publisher_point_2d, distance_buffer_2d);
            Topic_data_2d.publishScanLaser(frame_id, publisher_laserscan, scan_start_time, distance_buffer_2d);
        }
        else if (total_packet_data_[PAYLOAD_HEADER] == PACKET_HEADER_3D)
        {
            TransformPayload.getDistanceArray3D(&total_packet_data_[PAYLOAD_DATA], distance_buffer_3d);

            Topic_data_3d.publishScanImage(frame_id, publisher_image, distance_buffer_3d);
            Topic_data_3d.publishPoint3D(frame_id, publisher_point_3d, pointcloud_maker, distance_buffer_3d);
        }
        else if (total_packet_data_[PAYLOAD_HEADER] == PACKET_HEADER_DEV_INFO)
        {
            RCLCPP_INFO(this->get_logger(), "[F/W VERSION] %d.%d.%d", total_packet_data_[6], total_packet_data_[7], total_packet_data_[8]);
            RCLCPP_INFO(this->get_logger(), "[H/W VERSION] %d.%d.%d", total_packet_data_[9], total_packet_data_[10], total_packet_data_[11]);
        }
    }

    void CygLiDAR_D1::sendPacketData(cyglidar_serial &serial_port_)
    {
        serial_port_.requestDeviceInfo();
        std::this_thread::sleep_for(3s);
        // sleep for 3s, by requsting the info data.

        RCLCPP_INFO(this->get_logger(), "%s", serial_port_.requestRunMode(static_cast<eRunMode>(run_mode)).c_str());

        serial_port_.requestDurationControl(static_cast<eRunMode>(run_mode), duration_mode, duration_value);
        std::this_thread::sleep_for(1s);
        RCLCPP_INFO(this->get_logger(), "[PACKET REQUEST] PULSE DURATION : %d", duration_value);
        // sleep for a sec, by the duration

        serial_port_.requestFrequencyChannel(frequency_channel);
        RCLCPP_INFO(this->get_logger(), "[PACKET REQUEST] FREQUENCY CH.%d", frequency_channel);
    }
}
