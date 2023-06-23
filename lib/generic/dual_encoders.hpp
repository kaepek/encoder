#include "types.hpp"

#ifndef KAEPEK_GENERIC_DUAL_ENCODER
#define KAEPEK_GENERIC_DUAL_ENCODER

    /**
     * Generic encoder driver for reading sensor values from two encoders simultaneously.
     */
    class DualEncoder
    {
    public:
        /**
         * DualEncoder default constructor.
         *
         * @return Instance of DualEncoder class.
         */
        DualEncoder();

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