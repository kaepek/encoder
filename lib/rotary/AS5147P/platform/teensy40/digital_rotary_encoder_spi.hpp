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
        // need virtual methods again?
                /**
         * Method to setup the digital encoder such that it is ready to read measurement values via invocation of the read method.
         *
         */
        void setup() override;

        /**
         * Method to retrieve digital encoder measurement value.
         *
         * @return The encoder's current value.
         */
        uint32_t read() override;

        /**
         * Method to retrieve digital encoder measurement value.
         *
         * @param value reference to the value variable in which to populate the encoder's current measurement value.
         */
        void read(uint32_t &value) override;

        /**
         * Method to retrieve digital encoder measurement value.
         *
         * @param value reference to the value variable in which to populate the encoder's current measurement value.
         * @param parity reference to the parity variable in which to populate the encoder's current parity value.
         */
        void read(uint32_t &value, bool &parity) override;
    };

#endif

}
