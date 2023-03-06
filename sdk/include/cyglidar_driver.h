#ifndef CYGLIDAR_DRIVER_H_
#define CYGLIDAR_DRIVER_H_
#include "stdint.h"

namespace cyg_driver
{
    class TransformPayload
    {
        public:
            void getDistanceArray2D(uint8_t *payload_data_buffer_2d, int packet_total_length_2d, uint16_t *distance_value_array_buffer_2d);
            void getDistanceArray3D(uint8_t *payload_data_buffer_3d, int packet_total_length_3d, uint16_t *distance_value_array_buffer_3d);

        private:
    };
}

#endif
