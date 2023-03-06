#include "cyglidar_driver.h"
#include "serial.h"
#include "d_series_constant.h"

namespace cyg_driver
{        
    // Convert 2D, 3D rawdata to measured distance appending both separated data (LSB and MSB)
    void TransformPayload::getDistanceArray2D(uint8_t *payload_data_buffer_2d, int packet_total_length_2d, uint16_t *distance_value_array_buffer_2d)
    {
        int distance_count_2d = 0;

        for (int data_length = 0; data_length < (packet_total_length_2d - PAYLOAD_SIZE) - 1; data_length += 2)
        {
            uint8_t MSB = payload_data_buffer_2d[data_length];
            uint8_t LSB = payload_data_buffer_2d[data_length + 1];

            uint16_t raw_distance = ((MSB << 8) | LSB);

            distance_value_array_buffer_2d[distance_count_2d++] = raw_distance;
        }
    }

    void TransformPayload::getDistanceArray3D(uint8_t *payload_data_buffer_3d, int packet_total_length_3d, uint16_t *distance_value_array_buffer_3d)
    {
        int distance_count_3d = 0;
        for (int data_length = 0; data_length < (packet_total_length_3d - PAYLOAD_SIZE) - 1; data_length += 3)
        {
            uint8_t FIRST = payload_data_buffer_3d[data_length];
            uint8_t SECOND = payload_data_buffer_3d[data_length + 1];
            uint8_t THIRD = payload_data_buffer_3d[data_length + 2];

            // data1 is a combination of the first and left half of second
            // data2 is of the right half of second and third
            uint16_t data1 = (uint16_t)((FIRST << 4) | (SECOND >> 4));
            uint16_t data2 = (uint16_t)(((SECOND & 0xf) << 8) | THIRD);

            distance_value_array_buffer_3d[distance_count_3d++] = data1;
            distance_value_array_buffer_3d[distance_count_3d++] = data2;
        }
    }
}
