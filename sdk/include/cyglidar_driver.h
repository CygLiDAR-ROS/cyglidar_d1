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
            uint16_t raw_data;
            void getDistanceArray2D(uint8_t* _payload_buffer_2d, uint16_t* _distance_2d);

            uint8_t FIRST, SECOND, THIRD;
            uint16_t data1, data2;
            void getDistanceArray3D(uint8_t* _payload_buffer_3d, uint16_t* _distance_3d);

        private:
    };
}

#endif
