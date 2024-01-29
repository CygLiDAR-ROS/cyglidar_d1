#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "CYG_SerialUart.h"
#include "CYG_Driver.h"
#include "CYG_Constant.h"
#include "Topic2D.h"
#include "Topic3D.h"

using namespace std::chrono_literals;

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

        Topic2D*        topic_2d;
        Topic3D*        topic_3d;
        CYG_SerialUart* serial_port;
        CYG_Driver*     cyg_driver;

        rclcpp::Time start_time_scan_2d;

        std::string port_number;
        uint8_t     baud_rate_mode;
        std::string frame_id;
        uint8_t     run_mode;
        uint8_t     duration_mode;
        uint16_t    duration_value;
        uint8_t     frequency_channel;
        uint8_t     color_mode;

        std::thread double_buffer_thread;
        std::thread publish_thread;

        std::shared_future<void> future;
        std::promise<void> exit_signal;
        std::future_status status;

        std::string mode_notice;
        bool  info_flag = false;

        uint8_t packet_structure[D1_Const::SCAN_MAX_SIZE];
        uint8_t first_total_packet_data[D1_Const::SCAN_MAX_SIZE];
        uint8_t second_total_packet_data[D1_Const::SCAN_MAX_SIZE];
        uint16_t distance_buffer_2d[DATA_LENGTH_2D];
        uint16_t distance_buffer_3d[DATA_LENGTH_3D];

        uint8_t  publish_done_flag;
        uint8_t  publish_data_state;
        uint8_t  double_buffer_index;
        uint8_t  parser_return;
        uint16_t number_of_data;
};
