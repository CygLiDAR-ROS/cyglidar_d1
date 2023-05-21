#ifndef __D1_NODE_H
#define __D1_NODE_H

#include "serial.h"
#include "point_cloud_maker.h"
#include "d_series_constant.h"
#include "cyglidar_driver.h"
#include "point_cloud_maker.h"
#include "d1_2d_topic.h"
#include "d1_3d_topic.h"

#include <ros/ros.h>

namespace D1
{
    class CygLiDAR_D1
    {
        protected:
            std::string port;
            int baud_rate;
            std::string frame_id;
            int run_mode;
            int duration_mode;
            int duration_value;
            int frequency_channel;

            double scan_duration;

            uint8_t total_packet_data[SCAN_MAX_SIZE];
            uint8_t packet_structure[SCAN_MAX_SIZE];

            uint16_t distance_buffer_2d[cyg_driver::DATA_LENGTH_2D];
            uint16_t distance_buffer_3d[cyg_driver::DATA_LENGTH_3D];
            uint16_t number_of_data;

            ros::Publisher publisher_image;
            ros::Publisher publisher_laserscan;
            ros::Publisher publisher_point_2d;
            ros::Publisher publisher_point_3d;

            ros::NodeHandle nh;

        public:
            explicit CygLiDAR_D1(ros::NodeHandle nh_);
            void initParam();
            void onInit();
            void sendPacketData(cyglidar_serial &serial_port_);
            void publishLoop(cyglidar_serial &serial_port_);
            void publishData(uint8_t *total_packet_data_);

    };
}

#endif