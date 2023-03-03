#include "cyglidar_driver.h"
#include "serial.h"
#include "d_series_constant.h"

namespace cyg_driver
{       
    // Convert 2D, 3D rawdata to measured distance appending both separated data (LSB and MSB) 
    void TransformPayload::getDistanceArray2D(uint8_t *PAYLOAD_DATA_BUFFER_2D, int Packet_Total_Length_2D, float *Distance_ValueBuffer_2D)
    {
        int DistanceCount_2D = 0;

        for (int dataLength = 0; dataLength < (Packet_Total_Length_2D - PAYLOAD_SIZE) - 1; dataLength += 2)
        {
            uint8_t MSB = PAYLOAD_DATA_BUFFER_2D[dataLength];
            uint8_t LSB = PAYLOAD_DATA_BUFFER_2D[dataLength + 1];

            float rawDistance = (float)((MSB << 8) | LSB);

            Distance_ValueBuffer_2D[DistanceCount_2D++] = rawDistance;
        }
    }

    
    void TransformPayload::getDistanceArray3D(uint8_t *PAYLOAD_DATA_BUFFER_3D, int Packet_Total_Length_3D, uint16_t *DistanceValueArray_Buffer_3D)
    {
        int distanceCnt_3D = 0;
        for (int dataLength = 0; dataLength < (Packet_Total_Length_3D - PAYLOAD_SIZE) - 1; dataLength+=3)
        {
            uint8_t FIRST = PAYLOAD_DATA_BUFFER_3D[dataLength];
            uint8_t SECOND = PAYLOAD_DATA_BUFFER_3D[dataLength + 1];
            uint8_t THIRD = PAYLOAD_DATA_BUFFER_3D[dataLength + 2];

            // data1 is a combination of the first and left half of second
            // data2 is of the right half of second and third
            uint16_t data1 = (uint16_t)((FIRST << 4) | (SECOND >> 4));
            uint16_t data2 = (uint16_t)(((SECOND & 0xf) << 8) | THIRD);

            DistanceValueArray_Buffer_3D[distanceCnt_3D++] = data1;
            DistanceValueArray_Buffer_3D[distanceCnt_3D++] = data2;
        }
    }
}
