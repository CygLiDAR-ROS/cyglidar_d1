#include "d1_2d_topic.h"

namespace D1
{
    void Topic_2D::publishScanLaser(ros::Publisher publisher_laserscan_, sensor_msgs::LaserScan::Ptr message_laserscan_, std::string frame_id_,
                                    ros::Time start_, double scan_time_, uint16_t *distance_buffer_2d_)
    {
        message_laserscan_->header.frame_id = frame_id_;
        message_laserscan_->header.stamp = start_;
        message_laserscan_->angle_min = -static_cast<double>(CygLiDARD1::Sensor::HorizontalAngle / 2.0f * CygLiDARD1::Util::ToRadian);
        message_laserscan_->angle_max =  static_cast<double>(CygLiDARD1::Sensor::HorizontalAngle / 2.0f * CygLiDARD1::Util::ToRadian);
        message_laserscan_->angle_increment = static_cast<double>(CygLiDARD1::Sensor::AngleIncremet2D * CygLiDARD1::Util::ToRadian);
        message_laserscan_->scan_time = scan_time_;
        message_laserscan_->range_min = static_cast<double>(CygLiDARD1::Distance::Mode2D::Minimum_Depth_2D * CygLiDARD1::Util::MM_To_M);
        message_laserscan_->range_max = static_cast<double>(CygLiDARD1::Distance::Mode2D::Maximum_Depth_2D * CygLiDARD1::Util::MM_To_M);
        message_laserscan_->ranges.resize(cyg_driver::DATA_LENGTH_2D);
        message_laserscan_->intensities.resize(cyg_driver::DATA_LENGTH_2D);

        for (int i = 0; i < cyg_driver::DATA_LENGTH_2D; i++)
        {
            int data_index = (cyg_driver::DATA_LENGTH_2D - 1 - i);
            if (distance_buffer_2d_[data_index] < (float)(CygLiDARD1::Distance::Mode2D::Maximum_Depth_2D))
            {
                message_laserscan_->ranges[i] = distance_buffer_2d_[data_index] * MM2M;
            }
            else
            {
                message_laserscan_->ranges[i] = std::numeric_limits<float>::infinity();
            }
        }
        publisher_laserscan_.publish(message_laserscan_);
    }

    void Topic_2D::publishPoint2D(ros::Publisher publisher_point_2d_, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr message_point_2d_,
                                  std::string frame_id_, uint16_t *distance_buffer_2d_)
    {
        message_point_2d_->header.frame_id = frame_id_;
        message_point_2d_->is_dense = false;
        message_point_2d_->points.resize(cyg_driver::DATA_LENGTH_2D);

        double angle_increment_steps = static_cast<double>(CygLiDARD1::Sensor::AngleIncremet2D);

        int data_idx;
        uint16_t raw_distance;
        float point_angle_2d, point_angle_variable_2d;
        float world_coordinate_x, world_coordinate_y;

        for (int i = 0; i < cyg_driver::DATA_LENGTH_2D; i++)
        {
            // Reverse data order of the array
            data_idx = (cyg_driver::DATA_LENGTH_2D - 1 - i);

            raw_distance = (distance_buffer_2d_[data_idx]);

            point_angle_2d = (float)(((-HORIZONTAL_ANGLE / 2)+ point_angle_variable_2d) * CygLiDARD1::Util::ToRadian);
            point_angle_variable_2d += angle_increment_steps;

            world_coordinate_x = (sin(point_angle_2d) * raw_distance);
            world_coordinate_y = (cos(point_angle_2d) * raw_distance);

            pointcloud_2d_->points[i].x = world_coordinate_y * MM2M;
            pointcloud_2d_->points[i].y = world_coordinate_x * MM2M;
            pointcloud_2d_->points[i].z = 0.0;

            if (distance_buffer_2d_[data_idx] < CygLiDARD1::Distance::Mode2D::Maximum_Depth_2D)
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
        pcl_conversions::toPCL(ros::Time::now(), message_point_2d_->header.stamp);
        publisher_point_2d_.publish(message_point_2d_);
    }
}
