#include "serial.h"
#include "point_cloud_maker.h"
#include "d_series_constant.h"
#include "cyglidar_driver.h"

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

cyg_driver::TransformPayload TransformPayload;
ColorRGB colorRGB;

void publishMessageLaserScan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr &publisher_laserscan_,
                             sensor_msgs::msg::LaserScan::SharedPtr message_laserscan_, std::string frame_id_,
                             rclcpp::Time start_, double scan_time_, int payload_data_length_2d_, uint16_t *distance_value_array_buffer_2d_)
{
    message_laserscan_->header.frame_id = frame_id_;
    message_laserscan_->header.stamp = start_;
    message_laserscan_->angle_min = -static_cast<double>(CygLiDARD1::Sensor::HorizontalAngle / 2.0f * CygLiDARD1::Util::ToRadian);
    message_laserscan_->angle_max = static_cast<double>(CygLiDARD1::Sensor::HorizontalAngle / 2.0f * CygLiDARD1::Util::ToRadian);
    message_laserscan_->angle_increment = static_cast<double>(CygLiDARD1::Sensor::AngleIncremet2D * CygLiDARD1::Util::ToRadian);
    message_laserscan_->time_increment = (scan_time_ / (float)(payload_data_length_2d_ - 1));
    message_laserscan_->scan_time = scan_time_;
    message_laserscan_->range_min = static_cast<double>(CygLiDARD1::Distance::Mode2D::Minimum_Depth_2D * CygLiDARD1::Util::MM_To_M);
    message_laserscan_->range_max = static_cast<double>(CygLiDARD1::Distance::Mode2D::Maximum_Depth_2D * CygLiDARD1::Util::MM_To_M);
    message_laserscan_->ranges.resize(payload_data_length_2d_);
    message_laserscan_->intensities.resize(payload_data_length_2d_);

    for (int i = 0; i < payload_data_length_2d_; i++)
    {
        int data_index = (payload_data_length_2d_ - 1 - i);
        if (distance_value_array_buffer_2d_[data_index] < (float)(CygLiDARD1::Distance::Mode2D::Maximum_Depth_2D))
        {
            message_laserscan_->ranges[i] = distance_value_array_buffer_2d_[data_index] * MM2M;
        }
        else
        {
            message_laserscan_->ranges[i] = std::numeric_limits<float>::infinity();
        }
    }
    publisher_laserscan_->publish(*message_laserscan_);
}

void publishMessagePoint2D(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher_point_2d_,
                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud_2d_, std::string frame_id_,
                           int payload_data_length_2d_, uint16_t *distance_value_array_buffer_2d_)
{
    pointcloud_2d_->header.frame_id = frame_id_;
    pointcloud_2d_->is_dense = false;
    pointcloud_2d_->points.resize(payload_data_length_2d_);

    double angle_step_2d = static_cast<double>(CygLiDARD1::Sensor::AngleIncremet2D);
    float point_angle_var_2D = 0.0;
    float tempX_2D, tempY_2D;

    for (int i = 0; i < payload_data_length_2d_; i++)
    {
        // Reverse data order of the array
        int data_idx = (payload_data_length_2d_ - 1 - i);

        float actual_distance = (distance_value_array_buffer_2d_[data_idx]);

        float point_angle_2D = (float)(((-HORIZONTAL_ANGLE / 2)+ point_angle_var_2D) * CygLiDARD1::Util::ToRadian);
        point_angle_var_2D += angle_step_2d;

        float actualX = (sin(point_angle_2D) * actual_distance);
        float actualY = (cos(point_angle_2D) * actual_distance);

        tempX_2D = actualX;
        tempY_2D = actualY;

        float rotation_angle = -90.0f * CygLiDARD1::Util::ToRadian;

        actualX = (tempX_2D * cos(rotation_angle)) + (tempY_2D * -sin(rotation_angle));
        actualY = (tempX_2D * sin(rotation_angle)) + (tempY_2D * cos(rotation_angle));

        pointcloud_2d_->points[i].x = actualX * MM2M;
        pointcloud_2d_->points[i].y = -actualY * MM2M;
        pointcloud_2d_->points[i].z = 0.0;

        if (distance_value_array_buffer_2d_[data_idx] < CygLiDARD1::Distance::Mode2D::Maximum_Depth_2D)
        {
            pointcloud_2d_->points[i].r = 255;
            pointcloud_2d_->points[i].g = 255;
            pointcloud_2d_->points[i].b = 0;
            pointcloud_2d_->points[i].a = 255;
        }
        else
        {
            // Turn data invisible when it's greater than the maximum
            pointcloud_2d_->points[i].a = 0;
        }
    }
    pcl_conversions::toPCL(rclcpp::Clock().now(), pointcloud_2d_->header.stamp);

    sensor_msgs::msg::PointCloud2 message_point_cloud_2d;
    pcl::toROSMsg(*pointcloud_2d_, message_point_cloud_2d);
    publisher_point_2d_->publish(message_point_cloud_2d);
}

void publishMessagePoint3D(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher_point_3d_,
                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud_3d_, std::string frame_id_,
                           int payload_data_length_3d_, PointCloudMaker &PointCloud, uint16_t *distance_value_array_buffer_3d_)
{
    pointcloud_3d_->header.frame_id = frame_id_;
    pointcloud_3d_->is_dense = false;
    pointcloud_3d_->width = CygLiDARD1::Sensor::Width;
    pointcloud_3d_->height = CygLiDARD1::Sensor::Height;
    pointcloud_3d_->points.resize(payload_data_length_3d_);

    int buffer_index = 0;
    float position_x, position_y, position_z;
    for (int y = 0; y < CygLiDARD1::Sensor::Height; y++)
    {
        for (int x = 0; x < CygLiDARD1::Sensor::Width; x++)
        {
            buffer_index = x + (CygLiDARD1::Sensor::Width * y);
            uint16_t distance = distance_value_array_buffer_3d_[buffer_index];

            if(distance < CygLiDARD1::Distance::Mode3D::Maximum_Depth_3D)
            {
                if(PointCloud.calcPointCloud(distance, buffer_index, position_x, position_y, position_z) == eCalculationStatus::SUCCESS)
                {
                    pointcloud_3d_->points[buffer_index].x = position_z * MM2M;
                    pointcloud_3d_->points[buffer_index].y = -position_x * MM2M;
                    pointcloud_3d_->points[buffer_index].z = -position_y * MM2M;
                    uint32_t color_change_with_height = colorRGB.color_map[((int)position_y / 2) % colorRGB.color_map.size()];
                    pointcloud_3d_->points[buffer_index].rgb = *reinterpret_cast<float*>(&color_change_with_height);
                    pointcloud_3d_->points[buffer_index].a = 255;
                }
                else
                {
                    pointcloud_3d_->points[buffer_index].x = 0;
                    pointcloud_3d_->points[buffer_index].y = 0;
                    pointcloud_3d_->points[buffer_index].z = 0;
                    pointcloud_3d_->points[buffer_index].rgb = 0;
                    pointcloud_3d_->points[buffer_index].a = 0;
                }
            }
            else
            {
                    pointcloud_3d_->points[buffer_index].x = 0;
                    pointcloud_3d_->points[buffer_index].y = 0;
                    pointcloud_3d_->points[buffer_index].z = 0;
                    pointcloud_3d_->points[buffer_index].rgb = 0;
                    pointcloud_3d_->points[buffer_index].a = 0;
            }
        }
    }
    pcl_conversions::toPCL(rclcpp::Clock().now(), pointcloud_3d_->header.stamp);

    sensor_msgs::msg::PointCloud2 message_point_cloud_3d;
    pcl::toROSMsg(*pointcloud_3d_, message_point_cloud_3d);
    publisher_point_3d_->publish(message_point_cloud_3d);
}

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

    auto node = rclcpp::Node::make_shared("CyglidarNode");

    node->declare_parameter("port");
    node->declare_parameter("baud_rate");
    node->declare_parameter("frame_id");
    node->declare_parameter("run_mode");
    node->declare_parameter("duration_mode");
    node->declare_parameter("duration_value");
    node->declare_parameter("frequency_channel");

    node->get_parameter_or<std::string>("port", port, "/dev/ttyUSB0");
    node->get_parameter_or<int>("baud_rate", baud_rate, 3000000);
    node->get_parameter_or<std::string>("frame_id", frame_id, "laser_link");
    node->get_parameter_or<int>("run_mode", run_mode, 2);
    node->get_parameter_or<int>("duration_mode", duration_mode, PULSE_AUTO);
    node->get_parameter_or<int>("duration_value", duration_value, 10000);
    node->get_parameter_or<int>("frequency_channel", frequency_channel, 0);

    bool ready_publish = false;
    bool complete_set_buffer_2d = false;
    bool complete_set_buffer_3d = false;

    int packet_total_length_2d = 0;
    int payload_data_length_2d = 0;
    int packet_total_length_3d = 0;
    int payload_data_length_3d = 0;

    uint8_t  *payload_data_buffer_2d = 0;
    uint16_t *distance_value_array_buffer_2d = 0;
    uint8_t  *payload_data_buffer_3d = 0;
    uint16_t distance_value_array_buffer_3d[CygLiDARD1::Sensor::numPixel];

    // Call the following function so as to store colors to draw 3D data
    colorRGB.initColorMap();

    float param_x[CygLiDARD1::Sensor::numPixel], param_y[CygLiDARD1::Sensor::numPixel], param_z[CygLiDARD1::Sensor::numPixel];

    PointCloudMaker pointcloud_3d = PointCloudMaker(param_x, param_y, param_z, CygLiDARD1::Sensor::numPixel);

    pointcloud_3d.initLensTransform(CygLiDARD1::Sensor::PixelRealSize, CygLiDARD1::Sensor::Width, CygLiDARD1::Sensor::Height,
                                    CygLiDARD1::Parameter::OffsetCenterPoint_x, CygLiDARD1::Parameter::OffsetCenterPoint_y);

    uint8_t total_packet_data[SCAN_MAX_SIZE];

    boost::asio::io_service io;

    try
    {
        // Open the port
        cyglidar_pcl cyglidar_serial_port(port, baud_rate, io);

        auto publisher_laserscan = node->create_publisher<sensor_msgs::msg::LaserScan>("scan_laser", SCAN_MAX_SIZE);
        auto publisher_point_2d  = node->create_publisher<sensor_msgs::msg::PointCloud2>("scan_2D", 1);
        auto publisher_point_3d  = node->create_publisher<sensor_msgs::msg::PointCloud2>("scan_3D", 1);

        RCLCPP_INFO(node->get_logger(), "%s", cyglidar_serial_port.requestRunMode(static_cast<eRunMode>(run_mode)).c_str());

        RCLCPP_INFO(node->get_logger(), "[PACKET UPDATED] PULSE DURATION : %d", duration_value);
        cyglidar_serial_port.requestDurationControl(static_cast<eRunMode>(run_mode), duration_mode, duration_value);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // sleep for a sec, by the duration

        RCLCPP_INFO(node->get_logger(), "[PACKET UPDATED] FREQUENCY CH.%d", frequency_channel);
        cyglidar_serial_port.requestFrequencyChannel(frequency_channel);

        auto scan_laser = std::make_shared<sensor_msgs::msg::LaserScan>();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_2D(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_3D(new pcl::PointCloud<pcl::PointXYZRGBA>);

        while (rclcpp::ok())
        {
            rclcpp::Time scan_start_time;
            rclcpp::Time scan_end_time;
            double scan_duration;

            uint8_t packet_structure[SCAN_MAX_SIZE];
            uint16_t number_of_data = cyglidar_serial_port.getPacketLength(packet_structure, SCAN_MAX_SIZE);

            if (number_of_data > 0)
            {
                for (int i = 0; i < number_of_data; i++)
                {
                    if(CygParser(total_packet_data, packet_structure[i]) == CHECKSUM_PASSED)
                    {
                        if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_2D)
                        {
                            if (!complete_set_buffer_2d)
                            {// init 2d variable
                                packet_total_length_2d = (int)(total_packet_data[PAYLOAD_LENGTH_MSB] << 8 | total_packet_data[PAYLOAD_LENGTH_LSB]) + PAYLOAD_SIZE;
                                payload_data_length_2d = (int)((float)(packet_total_length_2d - PAYLOAD_SIZE) / 2.0);
                                distance_value_array_buffer_2d = new uint16_t[payload_data_length_2d];
                                payload_data_buffer_2d = new uint8_t[packet_total_length_2d - PAYLOAD_SIZE];

                                complete_set_buffer_2d = true;
                            }
                            if (complete_set_buffer_2d)
                            {
                                scan_start_time = node->now() - rclcpp::Duration(0, 150000); //nanosec
                                scan_end_time = scan_start_time;
                                scan_duration = (scan_start_time - scan_end_time).seconds();

                                payload_data_buffer_2d = &total_packet_data[PAYLOAD_DATA];

                                if (!ready_publish)
                                {
                                    ready_publish = true;
                                    TransformPayload.getDistanceArray2D(payload_data_buffer_2d, packet_total_length_2d, distance_value_array_buffer_2d);
                                    publishMessageLaserScan(publisher_laserscan, scan_laser, frame_id, scan_start_time, scan_duration, payload_data_length_2d, distance_value_array_buffer_2d);
                                    publishMessagePoint2D(publisher_point_2d, scan_2D, frame_id, payload_data_length_2d, distance_value_array_buffer_2d);
                                }
                                ready_publish = false;
                            }
                        }
                        else if (total_packet_data[PAYLOAD_HEADER] == PACKET_HEADER_3D)
                        {
                            if (!complete_set_buffer_3d)
                            {// init 3d variable
                                packet_total_length_3d = (int)(total_packet_data[PAYLOAD_LENGTH_MSB] << 8 | total_packet_data[PAYLOAD_LENGTH_LSB]) + PAYLOAD_SIZE;
                                float byteset_ratio_3d = (2.0 / 3.0);
                                payload_data_length_3d = (int)((float)(packet_total_length_3d - PAYLOAD_SIZE) * byteset_ratio_3d);
                                payload_data_buffer_3d = new uint8_t[packet_total_length_3d - PAYLOAD_SIZE];

                                complete_set_buffer_3d = true;
                            }
                            if (complete_set_buffer_3d)
                            {
                                payload_data_buffer_3d = &total_packet_data[PAYLOAD_DATA];
                                if (!ready_publish)
                                {
                                    ready_publish = true;
                                    TransformPayload.getDistanceArray3D(payload_data_buffer_3d, packet_total_length_3d, distance_value_array_buffer_3d);
                                    publishMessagePoint3D(publisher_point_3d, scan_3D, frame_id, payload_data_length_3d, pointcloud_3d, distance_value_array_buffer_3d);
                                }
                                ready_publish = false;
                            }
                        }
                    }
                    else
                    {
                        scan_start_time = node->now();
                    }
                }
            }
        }
        cyglidar_serial_port.close();
        RCLCPP_INFO(node->get_logger(), "PACKET UPDATED : STOP");
        delete total_packet_data;
        delete payload_data_buffer_2d;
        delete distance_value_array_buffer_2d;
        delete payload_data_buffer_3d;
        return 0;
    }
    catch (const boost::system::system_error& ex)
    {
        RCLCPP_ERROR(node->get_logger(), "An exception was thrown [%s]", ex.what());
        return -1;
    }
}
