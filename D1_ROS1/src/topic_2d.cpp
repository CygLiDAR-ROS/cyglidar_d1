#include "topic_2d.h"

using namespace Constant_D1;

void Topic2D::initPublisher(ros::Publisher _publisher_laserscan, ros::Publisher _publisher_point_2d)
{
    publisher_laserscan = _publisher_laserscan;
    publisher_point_2d  = _publisher_point_2d;
}

void Topic2D::assignLaserScan(std::string _frame_id)
{
    message_laserscan.header.frame_id = _frame_id;
    message_laserscan.angle_min = -Sensor::HorizontalAngle / 2.0f * Util::ToRadian;
    message_laserscan.angle_max =  Sensor::HorizontalAngle / 2.0f * Util::ToRadian;
    message_laserscan.angle_increment = Sensor::AngleIncremet2D * Util::ToRadian;
    message_laserscan.scan_time = 0;
    message_laserscan.range_min = static_cast<float>(Distance::Mode2D::Minimum_Depth_2D * Util::MM_To_M);
    message_laserscan.range_max = static_cast<float>(Distance::Mode2D::Maximum_Depth_2D * Util::MM_To_M);
    message_laserscan.ranges.resize(cyg_driver::DATA_LENGTH_2D);
    message_laserscan.intensities.resize(cyg_driver::DATA_LENGTH_2D);
}

void Topic2D::publishScanLaser(ros::Time _start_time, uint16_t *_distance_buffer_2d)
{
    message_laserscan.header.stamp = _start_time;

    for (uint8_t i = 0; i < cyg_driver::DATA_LENGTH_2D; i++)
    {
        // Reverse data order of the array
        buffer_index = (cyg_driver::DATA_LENGTH_2D - 1 - i);

        if (_distance_buffer_2d[buffer_index] < Distance::Mode2D::Maximum_Depth_2D)
        {
            message_laserscan.ranges[i] = _distance_buffer_2d[buffer_index] * MM2M;
        }
        else
        {
            message_laserscan.ranges[i] = std::numeric_limits<float>::infinity();
        }
    }
    publisher_laserscan.publish(message_laserscan);
}

void Topic2D::assignPCL2D(std::string _frame_id)
{
    pcl_2d.header.frame_id = _frame_id;
    pcl_2d.is_dense = false;
    pcl_2d.points.resize(cyg_driver::DATA_LENGTH_2D);
}

void Topic2D::publishPoint2D(uint16_t *_distance_buffer_2d)
{
    mappingPointCloud2D(_distance_buffer_2d);

    pcl_conversions::toPCL(ros::Time::now(), pcl_2d.header.stamp);

    publisher_point_2d.publish(pcl_2d);
}

void Topic2D::mappingPointCloud2D(uint16_t *_distance_buffer_2d)
{
    float point_2d_angle = 0;
    float point_2d_angle_variable = 0;

    for (uint8_t i = 0; i < cyg_driver::DATA_LENGTH_2D; i++)
    {
        buffer_index = (cyg_driver::DATA_LENGTH_2D - 1 - i);

        raw_distance = _distance_buffer_2d[buffer_index];

        point_2d_angle = ((-HORIZONTAL_ANGLE / 2)+ point_2d_angle_variable) * Util::ToRadian;
        point_2d_angle_variable += angle_increment_steps;

        camera_coordinate_x = (sin(point_2d_angle) * raw_distance);
        camera_coordinate_y = (cos(point_2d_angle) * raw_distance);

        pcl_2d.points[i].x = camera_coordinate_y * MM2M;
        pcl_2d.points[i].y = camera_coordinate_x * MM2M;
        pcl_2d.points[i].z = 0.0;

        if (_distance_buffer_2d[buffer_index] < Distance::Mode2D::Maximum_Depth_2D)
        {
            pcl_2d.points[i].rgba = 0xFFFFFF00; //ARGB(Yellow)
        }
        else
        {
            pcl_2d.points[i].a = 0; // Turn data invisible when it's greater than the maximum
        }
    }
}

