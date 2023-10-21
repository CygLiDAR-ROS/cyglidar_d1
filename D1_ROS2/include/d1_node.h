#ifndef __D1_NODE_H
#define __D1_NODE_H

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "cyglidar_serial.h"
#include "cyglidar_driver.h"
#include "d_series_constant.h"
#include "topic_2d.h"
#include "topic_3d.h"

using namespace std::chrono_literals;
using namespace Constant_D1;

class D1_Node : public rclcpp::Node
{
    public:
        explicit D1_Node();
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
        void convertData(received_data_buffer* _received_buffer);
        void convertInfoData(received_data_buffer* _received_buffer);
        void processDoubleBuffer();
        void runPublish();
        void doublebufferThread();
        void publishThread();

        std::shared_ptr<Topic2D>        topic_2d;
        std::shared_ptr<Topic3D>        topic_3d;
        std::shared_ptr<CyglidarSerial> serial_port;
        std::shared_ptr<CygDriver>      cyg_driver;

        std::string port;
        uint8_t baud_rate_mode;
        std::string frame_id;
        uint8_t run_mode;
        uint8_t duration_mode;
        uint16_t duration_value;
        uint8_t frequency_channel;
        uint8_t color_mode;
        uint16_t min_resolution;
        uint16_t max_resolution;

        rclcpp::Time scan_start_time;

        std::thread double_buffer_thread;
        std::thread publish_thread;

        std::shared_future<void> future;
        std::promise<void> exit_signal;
        std::future_status status;

        std::string mode_notice;
        bool info_flag = false;

        uint8_t publish_done_flag  = 0;
        uint8_t publish_data_state = 0;
        uint8_t double_buffer_index;
        uint8_t first_total_packet_data[SCAN_MAX_SIZE];
        uint8_t second_total_packet_data[SCAN_MAX_SIZE];
        uint8_t packet_structure[SCAN_MAX_SIZE];
        uint8_t parser_return;

        uint16_t number_of_data;
        uint16_t distance_buffer_2d[DATA_LENGTH_2D];
        uint16_t distance_buffer_3d[DATA_LENGTH_3D];

        const uint8_t PUBLISH_DONE = 0;
        const uint8_t PUBLISH_2D   = 1;
        const uint8_t PUBLISH_3D   = 2;

        const uint8_t PACKET_HEADER_2D       = 0x01;
        const uint8_t PACKET_HEADER_3D       = 0x08;
        const uint8_t PACKET_HEADER_DEV_INFO = 0x10;
};

#endif
