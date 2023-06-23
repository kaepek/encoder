#include <Arduino.h>
#include "imxrt.h"

namespace kaepek
{

/*#ifndef KAEPEK_GENERIC_ENCODER_CONSTS
#define KAEPEK_GENERIC_ENCODER_CONSTS
#endif*/
    
#ifndef KAEPEK_GENERIC_ENCODER_TYPES
#define KAEPEK_GENERIC_ENCODER_TYPES

    /**
     * DualEncodersSensorValues struct contains 2 integers: enc1_val (encoder 1 measurement value) and enc2_val (encoder 2 measurement value).
     **/
    struct DualEncodersSensorValues
    {
        uint32_t enc1_val;
        uint32_t enc2_val;
    };

#endif

}