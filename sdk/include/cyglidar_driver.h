#ifndef __CYGLIDAR_DRIVER_H_
#define __CYGLIDAR_DRIVER_H_
#include "stdint.h"

namespace cyg_driver
{
    constexpr uint16_t DATA_LENGTH_2D = 161;
    constexpr uint16_t DATA_LENGTH_3D = 9600;

    constexpr uint16_t PACKET_LENGTH_2D = DATA_LENGTH_2D * sizeof(uint16_t) + 1; //323
    constexpr uint16_t PACKET_LENGTH_3D = DATA_LENGTH_3D * 1.5 + 1;              //14401

    class TransformPayload
    {
        public:
            uint8_t MSB, LSB;
            uint16_t raw_distance;
            void getDistanceArray2D(uint8_t *payload_data_buffer_2d_, uint16_t *distance_2d_);

            uint8_t FIRST, SECOND, THIRD;
            uint16_t data1, data2;
            void getDistanceArray3D(uint8_t *payload_data_buffer_3d_, uint16_t *distance_3d_);

        private:
    };
}

#endif
