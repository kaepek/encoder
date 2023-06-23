#include "digital_encoder.hpp"

namespace kaepek
{

#ifndef KAEPEK_DUAL_DIGITAL_ENCODER
#define KAEPEK_DUAL_DIGITAL_ENCODER

    /**
     * Dual digital encoder driver for reading sensor values from two digital encoders simultaneously via their SPI interfaces.
     */
    class DualDigitalEncodersSPI
    {
    private:
        // Digital encoder SPI pin configuration for encoder 1.
        DigitalEncoderPinsSPI pins_enc1;
        // Digital encoder SPI pin configuration for encoder 2.
        DigitalEncoderPinsSPI pins_enc2;
        // Digital encoder 1 instance.
        DigitalEncoderSPI enc1;
        // Digital encoder 2 instance.
        DigitalEncoderSPI enc2;

    public:
        /**
         * DualDigitalEncodersSPI default constructor.
         *
         * @return Instance of DualDigitalEncodersSPI class.
         */
        DualDigitalEncodersSPI();

        /**
         * DualDigitalEncodersSPI constructor with parameters.
         *
         * @param enc1 instance of the DigitalEncoderSPI class for encoder 1.
         * @param enc2 instance of the DigitalEncoderSPI class for encoder 2.
         * @return Instance of DualDigitalEncodersSPI class.
         */
        DualDigitalEncodersSPI(DigitalEncoderSPI enc1, DigitalEncoderSPI enc2);

        /**
         * Method to setup both encoders such that they are ready to read measurement values via invocation of the read method.
         *
         */
        void setup();

        /**
         * Method to retrieve both encoder's current values.
         *
         * @return Both encoder's current values.
         */
        DualEncodersSensorValues read();

        /**
         * Method to retrieve both encoder's measurement values.
         *
         * @param value reference to the values variable in which to populate the both encoder's current measurement values.
         */
        void read(DualEncodersSensorValues &sensorValues);
    };

#endif

}
