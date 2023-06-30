#include "digital_rotary_encoder2.hpp"

namespace kaepek
{
#ifndef KAEPEK_DIGITAL_ROTARY_ENCODER_SPI_IMPL
#define KAEPEK_DIGITAL_ROTARY_ENCODER_SPI_IMPL

    template <typename T>
    void DigitalRotaryEncoderSPI<T>::DigitalRotaryEncoderSPI() : Encoder ()
    {
    }

    template <typename T>
    void DigitalRotaryEncoderSPI<T>::DigitalRotaryEncoderSPI(DigitalEncoderPinsSPI pins) : Encoder ()
    {
        this->pins = pins;
    }

    template <typename T>
    void DigitalRotaryEncoderSPI<T>::getPins()
    {
        return this->pins;
    }

    template <typename T>
    void DigitalRotaryEncoderSPI<T>::setup()
    {
        // Set pin modes.
        pinMode(this->pins.csn, OUTPUT);
        pinMode(this->pins.mosi, OUTPUT);
        pinMode(this->pins.miso, INPUT);
        pinMode(this->pins.sck, OUTPUT);
    };

#endif
}