#include "generic/types.hpp"
#include "generic/encoder.hpp"

namespace kaepek
{

#ifndef KAEPEK_DIGITAL_ENCODER_SPI_TYPES
#define KAEPEK_DIGITAL_ENCODER_SPI_TYPES

    /**
     * DigitalEncoderPinsSPI struct contains 4 integers: csn (chip select), sck (clock), miso (master input slave output) and mosi (master output slave input).
     **/
    struct DigitalEncoderPinsSPI
    {
        int csn;
        int sck;
        int miso;
        int mosi;
    };

#endif

#ifndef KAEPEK_DIGITAL_ROTARY_ENCODER_SPI
#define KAEPEK_DIGITAL_ROTARY_ENCODER_SPI

    /**
     * Digital encoder driver for reading sensor values from a single digital encoder via its SPI interface.
     */
    template <typename T>
    class DigitalRotaryEncoderSPI : public Encoder
    {

    protected:
        DigitalEncoderPinsSPI pins;

    public:
        /**
         * DigitalRotaryEncoderSPI default constructor.
         *
         * @return Instance of DigitalRotaryEncoderSPI class.
         */
        DigitalRotaryEncoderSPI();

        /**
         * DigitalRotaryEncoderSPI constructor with DigitalEncoderPinsSPI parameter.
         *
         * @param pins pin configuration for digital encoders with an SPI interface.
         * @return Instance of DigitalRotaryEncoderSPI class.
         */
        DigitalRotaryEncoderSPI(DigitalEncoderPinsSPI pins);

        /**
         * Method to retrieve the DigitalRotaryEncoderSPI instance's SPI interface pin configuration.
         *
         * @return Instance of the DigitalEncoderPinsSPI struct stored within the DigitalRotaryEncoderSPI instance as a private variable.
         */
        DigitalEncoderPinsSPI getPins();

        /**
         * Method to setup the digital encoder such that it is ready to read measurement values via invocation of the read method.
         *
         */
        virtual void setup();

        /**
         * Method to retrieve digital encoder measurement value.
         *
         * @return The encoder's current value.
         */
        virtual uint32_t read() = 0;

        /**
         * Method to retrieve digital encoder measurement value.
         *
         * @param value reference to the value variable in which to populate the encoder's current measurement value.
         */
        virtual void read(uint32_t &value) = 0;

        /**
         * Method to retrieve digital encoder measurement value.
         *
         * @param value reference to the value variable in which to populate the encoder's current measurement value.
         * @param parity reference to the parity variable in which to populate the encoder's current parity value.
         */
        virtual void read(uint32_t &value, bool &parity) = 0;
    };

#endif

}