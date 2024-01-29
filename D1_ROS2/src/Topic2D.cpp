#include "Topic2D.h"

void Topic2D::initPublisher(rclcpp::Publisher<LaserScan>::SharedPtr _publisher_laserscan, rclcpp::Publisher<PointCloud2>::SharedPtr _publisher_point_2d)
{
    publisher_laserscan = _publisher_laserscan;
    publisher_point_2d  = _publisher_point_2d;
}

void Topic2D::applyPointCloud2D(uint16_t* _distance_buffer_2d)
{
    point_2d_angle = 0;
    point_2d_angle_variable = 0;

    for (uint8_t i = 0; i < DATA_LENGTH_2D; i++)
    {
        buffer_index = (DATA_LENGTH_2D - 1 - i);

        raw_distance = _distance_buffer_2d[buffer_index];

        point_2d_angle = ((-D1_Const::HORIZONTAL_ANGLE / 2) + point_2d_angle_variable) * ROS_Const::DEGREE2RADIAN;
        point_2d_angle_variable += D1_Const::ANGLE_INCREMENT_2D;

        real_world_coordinate_x = (sin(point_2d_angle) * raw_distance);
        real_world_coordinate_y = (cos(point_2d_angle) * raw_distance);

        pcl_2d->points[i].x = real_world_coordinate_y * ROS_Const::MM2M;
        pcl_2d->points[i].y = real_world_coordinate_x * ROS_Const::MM2M;
        pcl_2d->points[i].z = 0.0;

        if (_distance_buffer_2d[buffer_index] < D1_Const::DISTANCE_MAX_VALUE_2D)
        {
            pcl_2d->points[i].rgba = 0xFFFFFF00; //ARGB(Yellow)
        }
        else
        {
            pcl_2d->points[i].a = 0; // Turn data invisible when it's greater than the maximum
        }
    }
}

void Topic2D::assignPCL2D(const std::string &_frame_id)
{
    message_point_cloud_2d = std::make_shared<PointCloud2>();
    pcl_2d.reset(new pcl_XYZRGBA());

    pcl_2d->header.frame_id = _frame_id;
    pcl_2d->is_dense        = false;
    pcl_2d->points.resize(DATA_LENGTH_2D);
}

void Topic2D::publishPoint2D()
{
    pcl_conversions::toPCL(rclcpp::Clock().now(), pcl_2d->header.stamp);

    pcl::toROSMsg(*pcl_2d, *message_point_cloud_2d); //change type from pointcloud(pcl) to ROS message(PointCloud2)
    publisher_point_2d->publish(*message_point_cloud_2d);
}

void Topic2D::assignLaserScan(const std::string &_frame_id)
{
    message_laserscan = std::make_shared<LaserScan>();

    message_laserscan->header.frame_id = _frame_id;
    message_laserscan->angle_min       = -D1_Const::HORIZONTAL_ANGLE / 2.0f * ROS_Const::DEGREE2RADIAN;
    message_laserscan->angle_max       =  D1_Const::HORIZONTAL_ANGLE / 2.0f * ROS_Const::DEGREE2RADIAN;
    message_laserscan->angle_increment =  D1_Const::ANGLE_INCREMENT_2D * ROS_Const::DEGREE2RADIAN;
    message_laserscan->scan_time       = 0;
    message_laserscan->range_min       = D1_Const::DISTANCE_MIN_VALUE_2D * ROS_Const::MM2M;
    message_laserscan->range_max       = D1_Const::DISTANCE_MAX_VALUE_2D * ROS_Const::MM2M;
    message_laserscan->ranges.resize(DATA_LENGTH_2D);
    message_laserscan->intensities.resize(DATA_LENGTH_2D);
}

void Topic2D::publishScanLaser(rclcpp::Time _start_time, uint16_t* _distance_buffer_2d)
{
    message_laserscan->header.stamp = _start_time;

    for (uint8_t i = 0; i < DATA_LENGTH_2D; i++)
    {
        // Reverse data order of the array
        buffer_index = (DATA_LENGTH_2D - 1 - i);

        if (_distance_buffer_2d[buffer_index] < D1_Const::DISTANCE_MAX_VALUE_2D)
        {
            message_laserscan->ranges[i] = _distance_buffer_2d[buffer_index] * ROS_Const::MM2M;
        }
        else
        {
            message_laserscan->ranges[i] = std::numeric_limits<float>::infinity();
        }
    }

    publisher_laserscan->publish(*message_laserscan);
}
