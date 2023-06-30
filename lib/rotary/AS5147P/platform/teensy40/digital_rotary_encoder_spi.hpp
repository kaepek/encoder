#include "generic/digital_rotary_encoder_spi.cpp"

namespace kaepek
{

#ifndef KAEPEK_DIGITAL_ROTARY_ENCODER_SPI_AS5147P
#define KAEPEK_DIGITAL_ROTARY_ENCODER_SPI_AS5147P

    class AS5147P
    {
    };

    template <>
    class DigitalRotaryEncoderSPI<AS5147P> : public DigitalRotaryEncoderSPI<AS5147P>
    {
    };

#endif

}
