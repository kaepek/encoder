#ifndef KAEPEK_GENERIC_ENCODER
#define KAEPEK_GENERIC_ENCODER

    /**
     * Generic encoder driver interface for reading sensor values from a single encoder.
     */
    class Encoder
    {
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
         * Encoder default constructor.
         *
         * @return Instance of Encoder class.
         */
        Encoder();

        /**
         * Method to setup the encoder such that it is ready to read measurement values via invocation of the read method.
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
         * Method to retrieve the encoder measurement value.
         *
         * @param value reference to the value variable in which to populate the encoder's current measurement value.
         */
        void read(uint32_t &value);

        /**
         * Method to calculate the difference between old and new encoder measurement values. Respects modular arithmetic of the encoder_divisions.
         *
         * @param old_value the encoder's old measurement value.
         * @param new_value the encoder's new measurement value.
         */
        double get_reading_delta(double old_value, double new_value);
    };

#endif