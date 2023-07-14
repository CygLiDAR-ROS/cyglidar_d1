#include "cyglidar_driver.h"

namespace cyg_driver
{
    // Convert 2D, 3D rawdata to measured distance appending both separated data (LSB and MSB)
    void TransformPayload::getDistanceArray2D(uint8_t *payload_data_buffer_2d_, uint16_t *distance_2d_)
    {
        uint8_t distance_count_2d = 0;

        for (int data_length = 0; data_length < PACKET_LENGTH_2D - 1; data_length += 2)
        {
            MSB = payload_data_buffer_2d_[data_length];
            LSB = payload_data_buffer_2d_[data_length + 1];

            raw_distance = (uint16_t)((MSB << 8) | LSB);

            distance_2d_[distance_count_2d++] = raw_distance;
        }
    }

    void TransformPayload::getDistanceArray3D(uint8_t *payload_data_buffer_3d_, uint16_t *distance_3d_)
    {
        uint16_t distance_count_3d = 0;

        for (int data_length = 0; data_length < PACKET_LENGTH_3D - 1; data_length += 3)
        {
            FIRST = payload_data_buffer_3d_[data_length];
            SECOND = payload_data_buffer_3d_[data_length + 1];
            THIRD = payload_data_buffer_3d_[data_length + 2];

            // data1 is a combination of the first and left half of second
            // data2 is of the right half of second and third
            data1 = (FIRST << 4) | (SECOND >> 4);
            data2 = ((SECOND & 0xf) << 8) | THIRD;

            distance_3d_[distance_count_3d++] = data1;
            distance_3d_[distance_count_3d++] = data2;
        }
    }
}
