#ifndef CYGLIDAR_DRIVER_H_
#define CYGLIDAR_DRIVER_H_
#include "stdint.h"

namespace cyg_driver
{
    constexpr uint16_t DATA_LENGTH_2D = 161;
    constexpr uint16_t DATA_LENGTH_3D = 9600;

    constexpr uint16_t PACKET_LENGTH_2D = DATA_LENGTH_2D * sizeof(uint16_t) + 1; //323
    constexpr uint16_t PACKET_LENGTH_3D = DATA_LENGTH_3D * 1.5 + 1;//14401

    class TransformPayload
    {
        public:
            void getDistanceArray2D(uint8_t *payload_data_buffer_2d, uint16_t *Distance2D);
            void getDistanceArray3D(uint8_t *payload_data_buffer_3d, uint16_t *Distance3D);

        private:
    };
}

#endif
