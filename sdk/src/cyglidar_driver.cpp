#include "cyglidar_driver.h"

// Convert 2D, 3D rawdata to measured distance appending both separated data (LSB and MSB)
void CygDriver::getDistanceArray2D(uint8_t* _payload_buffer_2d, uint16_t* _distance_2d)
{
    buffer_count_2d = 0;

    for (uint16_t data_length = 0; data_length < PACKET_LENGTH_2D - 1; data_length += 2)
    {
        MSB = _payload_buffer_2d[data_length];
        LSB = _payload_buffer_2d[data_length + 1];

        raw_data = (uint16_t)((MSB << 8) | LSB);

        _distance_2d[buffer_count_2d++] = raw_data;
    }
}

void CygDriver::getDistanceArray3D(uint8_t* _payload_buffer_3d, uint16_t* _distance_3d)
{
    buffer_count_3d = 0;

    for (uint16_t data_length = 0; data_length < PACKET_LENGTH_3D - 1; data_length += 3)
    {
        FIRST  = _payload_buffer_3d[data_length];
        SECOND = _payload_buffer_3d[data_length + 1];
        THIRD  = _payload_buffer_3d[data_length + 2];

        // data1 is a combination of the first and left half of second
        // data2 is of the right half of second and third
        data1 = (FIRST << 4) | (SECOND >> 4);
        data2 = ((SECOND & 0xf) << 8) | THIRD;

        _distance_3d[buffer_count_3d++] = data1;
        _distance_3d[buffer_count_3d++] = data2;
    }
}
