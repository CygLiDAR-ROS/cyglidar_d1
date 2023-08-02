#ifndef __D1_NODE_H
#define __D1_NODE_H

#include "serial.h"
#include "point_cloud_maker.h"
#include "d_series_constant.h"
#include "cyglidar_driver.h"
#include "topic_2d.h"
#include "topic_3d.h"

#include <ros/ros.h>
#include <thread>
#include <future>

class D1_Node
{
    public:
        explicit D1_Node(ros::NodeHandle _nh);
        virtual ~D1_Node();

        void connectBoostSerial();
        void disconnectBoostSerial();
        void loopCygParser();

    private:
        struct received_data_buffer
        {
            uint8_t* packet_data;
        }received_buffer[2];

        void initConfiguration();
        void requestPacketData();
        void processDoubleBuffer();
        void convertData(received_data_buffer *_received_buffer);
        void publishThread();
        void runPublish();

        cyglidar_serial *serial_port;
        Topic2D *topic_2d;
        Topic3D *topic_3d;

        std::string port;
        int baud_rate;
        std::string frame_id;
        int run_mode;
        int duration_mode;
        int duration_value;
        int frequency_channel;

        ros::NodeHandle nh;
        ros::Time scan_start_time;

        boost::asio::io_service io;

        std::thread publish_thread;
        std::shared_future<void> future;
        std::promise<void> exit_signal;

        uint8_t publish_done_flag  = 0;
        uint8_t publish_data_state = 0;
        uint8_t double_buffer_index;
        uint8_t first_total_packet_data[SCAN_MAX_SIZE];
        uint8_t second_total_packet_data[SCAN_MAX_SIZE];
        uint8_t packet_structure[SCAN_MAX_SIZE];
        uint8_t parser_return;

        uint16_t number_of_data;
        uint16_t distance_buffer_2d[cyg_driver::DATA_LENGTH_2D];
        uint16_t distance_buffer_3d[cyg_driver::DATA_LENGTH_3D];

        const uint8_t PUBLISH_DONE = 0;
        const uint8_t PUBLISH_2D   = 1;
        const uint8_t PUBLISH_3D   = 2;

        const uint8_t PACKET_HEADER_2D       = 0x01;
        const uint8_t PACKET_HEADER_3D       = 0x08;
        const uint8_t PACKET_HEADER_DEV_INFO = 0x10;
};

#endif
