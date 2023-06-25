#include "types.hpp"

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
    class DigitalRotaryEncoderSPI
    {

    private:
        DigitalEncoderPinsSPI pins;

    public:
        // A particular encoder's "encoder_divisions", aka the maximum modular value before starting back from zero.
        static const uint32_t encoder_divisions;
        // A particular encoder's "encoder_divisions" divided by 2, aka half the maximum modular value before starting back from zero.
        static const uint32_t encoder_divisions_over_2;
        // The maximum absolute value that an encoder can jump after one time step ahead, note the sample time must be sufficient or else it could jump more.
        static const double skip_tolerance;
        // The maximum number of times the measured difference between consecutive encoder value readings could exceed its tolerance before indicating an issue with the encoder.
        static const uint32_t skip_threshold;

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
        void setup();

        /**
         * Method to retrieve digital encoder measurement value.
         *
         * @return The encoder's current value.
         */
        uint32_t read();

        /**
         * Method to retrieve digital encoder measurement value.
         *
         * @param value reference to the value variable in which to populate the encoder's current measurement value.
         */
        void read(uint32_t &value);

        /**
         * Method to retrieve digital encoder measurement value.
         *
         * @param value reference to the value variable in which to populate the encoder's current measurement value.
         * @param parity reference to the parity variable in which to populate the encoder's current parity value.
         */
        void read(uint32_t &value, bool &parity);

        /**
         * Method to calculate the difference between old and new encoder measurement values. Respects modular arithmetic of the encoder_divisions.
         *
         * @param old_value the encoder's old measurement value.
         * @param new_value the encoder's new measurement value.
         */
        double get_reading_delta(double old_value, double new_value);
    };

#endif

}