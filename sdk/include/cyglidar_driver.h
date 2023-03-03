#ifndef CYGLIDAR_DRIVER_H_
#define CYGLIDAR_DRIVER_H_
#include "stdint.h"

namespace cyg_driver
{
    class TransformPayload
    {
        public:
            void getDistanceArray2D(uint8_t *Distance_SetBuffer_2D, int MEASURED_DATA_TOTAL_SIZE_2D, float *Distance_ValueBuffer_2D);
            void getDistanceArray3D(uint8_t *Distance_SetBuffer_3D, int MEASURED_DATA_TOTAL_SIZE_3D, uint16_t *DistanceBuffer);

        private:
    };
}

#endif
