#include <Arduino.h>
#include "imxrt.h"
#include "digital_rotary_encoder.hpp"

namespace kaepek
{

#ifndef KAEPEK_ROTARY_ENCODER_SAMPLE_VALIDATOR
#define KAEPEK_ROTARY_ENCODER_SAMPLE_VALIDATOR

    /**
     * Encoder sample validator class for a single encoder. Allows for safe usage of an encoder, as it validates that the received values
     * are plausable. Inherit from this class and then implement "post_sample_logic" and "post_fault_logic" methods.
     *
     * Logic implemented within the post_sample_logic(uint32_t encoder_value) is only fired if a sample is considered reliable.
     *
     * Logic implemented within the post_fault_logic(kaepek::RotaryEncoderSampleValidator::Fault fault_code) is fired if an issue with the encoder is detected, or if set_direction_enforcement method has be called with the bool enabled = true argument and it turned against the validated direction.
     */
    template <class T>
    class RotaryEncoderSampleValidator
    {
    public:
        /**
         * RotaryEncoderSampleValidator default constructor.
         *
         * @return Instance of RotaryEncoderSampleValidator class.
         */
        RotaryEncoderSampleValidator();

        /**
         * RotaryEncoderSampleValidator constructor with parameters.
         *
         * @param encoder
         * @param sample_period_microseconds
         * @return Instance of DualDigitalRotaryEncoderSampleValidatorEncoderSPI class.
         */
        RotaryEncoderSampleValidator(T encoder, float sample_period_microseconds);

        /**
         * RotaryEncoderSampleValidator Fault enum class type:
         * - WrongDirection: indicates that the encoder move against the configured RotaryEncoderSampleValidator direction for enough samples to trigger a fault.
         * - SkippedSteps: indicates that the the encoder tracker lost tracking for enough samples to trigger a fault.
         */
        enum class Fault
        {
            WrongDirection = 0,
            SkippedSteps = 1,
        };

        /**
         * ConfigurationIssue enum typedef:
         * A collection of configuration issue descriptions for the RotaryEncoderSampleValidator class indicating it has not been configured correctly and will not start successfully.
         * - DirectionNotSet: indicates one needs to invoke the set_direction method with a valid direction value.
         * - SkipToleranceNotSet: indicates one needs to invoke the set_skip_tolerance method with a valid tolerance value.
         * - SkipFaultThresholdNotSet: indicates one needs to invoke the set_skip_threshold method with a valid threshold value.
         * - DirectionEnforcementNotSet: indicates one needs to invoke the set_direction_enforcement method with a valid enabled value.
         * - WrongWayToleranceNotSet: indicates one needs to invoke the set_wrong_way_tolerance method with a valid tolerance value.
         * - WrongWayFaultThresholdNotSet: indicates one needs to invoke the set_wrong_way_threshold method with a valid threshold value.
         */
        class ConfigurationIssue
        {
        public:
            typedef enum
            {
                DirectionNotSet = 0,
                SkipToleranceNotSet = 1,
                SkipFaultThresholdNotSet = 2,
                DirectionEnforcementNotSet = 3,
                WrongWayToleranceNotSet = 4,
                WrongWayFaultThresholdNotSet = 5
            } Type;
        };

        /**
         * RotaryEncoderSampleValidator Direction enum class type:
         * - CounterClockwise
         * - Clockwise
         * - UnSet: indicates an invalid RotaryEncoderSampleValidator direction value.
         */
        enum class Direction
        {
            CounterClockwise = 0,
            Clockwise = 1,
            UnSet = 2
        };

        // Public direction methods.

        /**
         * Method to set the encoder validated direction.
         *
         * @param direction The direction in which the EncoderTracker will validate the encoder motion.
         */
        void set_direction(Direction direction);

        /**
         * Method to get the get direction in which the encoder motion will be validated against.
         *
         * @return The direction is which the encoder motion is validated against by the EncoderTracker.
         */
        Direction get_direction();

        /**
         * Method to set whether the direction of the encoder should be validated.
         *
         * @param enabled The value which describes whether or not the EncoderTracker should or shouldn't validation motion.
         */
        void set_direction_enforcement(bool enabled);

        /**
         * Method to get whether or not the EncoderTracker is going to validate the direction of the encoder.
         */
        bool get_direction_enforcement();

        // Public skip methods.

        /**
         * Method to set the skip tolerance of the EncoderTracker.
         *
         * @param tolerance The absolute value which the encoder is allowed to move between consecutive time steps without incuring an increase in the skipped_steps_ctr private variable.
         */
        void set_skip_tolerance(double tolerance);

        /**
         * Method to get the skip tolerance of the EncoderTracker.
         *
         * @return The absolute value which the encoder is allowed to move between consecutive time steps without incuring an increase in the skipped_steps_ctr private variable.
         */
        double get_skip_tolerance();

        /**
         * Method to set the skip threshold of the EncoderTracker.
         *
         * @param threshold The number of samples by the EncoderTracker where the motion of the encoder between consecutive time steps has exceeded the skip threshold, before it triggers a SkippedSteps fault.
         */
        void set_skip_threshold(uint32_t threshold);

        /**
         * Method to get the skip threshold of the EncoderTracker.
         *
         * @return The number of samples by the EncoderTracker where the motion of the encoder between consecutive time steps has exceeded the skip threshold, before it triggers a SkippedSteps fault.
         */
        uint32_t get_skip_threshold();

        // Public wrong way methods.

        /**
         * Method to set the wrong way tolerance of the EncoderTracker.
         *
         * @param tolerance The absolute value which the encoder is allowed to move in the wrong direction between consecutive time steps without incuring an increase in the wrong_direction_ctr private variable.
         */
        void set_wrong_way_tolerance(double tolerance);

        /**
         * Method to get the wrong direction tolerance of the EncoderTracker.
         *
         * @return The absolute value which the encoder is allowed to move in the wrong direction between consecutive time steps without incuring an increase in the wrong_direction_ctr private variable.
         */
        double get_wrong_direction_tolerance();

        /**
         * Method to set the wrong way threshold of the EncoderTracker.
         *
         * @param threshold The number of samples by the EncoderTracker where the motion of the encoder is allowed to move in the wrong direction, between consecutive time steps, such that it exceededs the wrong direction threshold and triggers a WrongDirection fault.
         */
        void set_wrong_way_threshold(uint32_t threshold);

        /**
         * Method to get the wrong way threshold of the EncoderTracker.
         *
         * @return The number of samples by the EncoderTracker where the motion of the encoder is allowed to move in the wrong direction, between consecutive time steps, such that it exceededs the wrong direction threshold and triggers a WrongDirection fault.
         */
        uint32_t get_wrong_way_threshold();

        /**
         * Method to setup the EncoderTracker.
         *
         * Ensures that the encoder setup method has been called and configures the sampling timer.
         */
        void setup();

        // Public start method.

        /**
         * Method to start the EncoderTracker.
         * Ensures that the EncoderTracker has been configured correctly and starts the sampling timer.
         *
         * @return The start status of the EncoderTracker, true for it started correctly or false indicating a configuration issue is present.
         */
        bool start();

        /**
         * Method to print via the microcontroller serial port what configuration issues have been detected by the EncoderTracker.
         */
        void print_configuration_issues();

        /**
         * Method to stop the EncoderTracker via disabling the sampling timer.
         */
        void stop();

        /**
         * Method to detect whether a new sample exists within the internal sample buffer and saves it to a cache for retrieval by invoking the get_sample_and_elapsed_time method.
         */
        bool has_new_sample();

        /**
         * Method to retrieve the latest encoder value and the time since the last sample was taken.
         *
         * @param encoder_value Reference to the variable which will be populated by the latest encoder measurement sample value when invoking the method.
         * @param micros_since_last_sample Reference to the variable which will be populated by the amount of time which has elapsed since the last encoder measurement sample.
         */
        void get_sample_and_elapsed_time(uint32_t &encoder_value, uint32_t &micros_since_last_sample);

        /**
         * Method to override in order to define bespoke logic to be performed when a successful sample is taken and is determined to be a plausible value and the value indicates motion which obeys the direction constraint (if direction enforcement has been enabled).
         *
         * @param encoder_value The latest validated encoder measurement sample value.
         */
        virtual void post_sample_logic(uint32_t encoder_value);

        /**
         * Method to override in order to define bespoke logic to be performed when the EncoderTracker detects an inplausabile measurement sample value or the motion of the encoder is determined to violate the motion direction contraint (if direction enforcement has been enabled).
         */
        virtual void post_fault_logic(Fault fault_code);

    private:
        // Donfiguration values.

        // The variable which stores the configured motion direction constraint if it has been enabled.
        Direction direction;
        // The variable which describes whether or direction enforcement has been enabled.
        bool direction_enforcement = false;
        // The configured tolerance for which the difference of consecutive encoder measurement values indicate backwards motion (to the configured direction) to the extent that the wrong_direction_ctr should be incremented.
        double wrong_way_tolerance = 0.0;
        // The configured threshold for which counts exceeding this value by the wrong_direction_ctr would indicate a fault.
        uint32_t wrong_way_threshold = 0;
        // The configured tolerance for which the absolute difference of consecutive encoder measurement values indicate motion beyond what would be expected while tracking and therefore the skipped_steps_ctr should be incremented.
        double skip_tolerance = 0.0;
        // The configured threshold for which counts exceeding this value by the skipped_steps_ctr would indicate a fault.
        uint32_t skip_threshold = 0;
        // the configured sample time between invocations of the private sample callback in microseconds.
        float sample_period_microseconds;

        // Configuration state.

        // Whether or not the direction has been successfully set.
        bool direction_set = false;
        // Whether or not the direction enforcement has been successfully set.
        bool direction_enforcement_set = false;
        // Whether or not the wrong way tolerance has been successfully set.
        bool wrong_way_tolerance_set = false;
        // Whether or not the wrong way threshold has been successfully set.
        bool wrong_way_threshold_set = false;
        // Whether or not the skip tolerance has been successfully set.
        bool skip_tolerance_set = false;
        // Whether or not the skip threshold has been successfully set.
        bool skip_threshold_set = false;

        // Configuration validation messages which will be written out the the serial out port if a configuration issues is detected by the EncoderTracker.
        char ConfigurationIssueMessage[6][87] = {
            "Must set a direction value using the method: set_direction",
            "Must set a skip tolerance value using the method: set_skip_tolerance",
            "Must set a skip fault threshold value using the method: set_skip_threshold",
            "Must set a the direction enforcement value using the method: set_direction_enforcement",
            "Must set a wrong way tolerance value using the method: set_wrong_way_tolerance",
            "Must set a wrong way fault threshold value using the method: set_wrong_way_threshold"};

        // Internal state.

        // Whether or not the EncoderTracker has started successfully.
        bool started = false;
        // Whether or not the EncoderTracker has detected a fault.
        volatile bool fault = false;
        // Whether or not we have one existing sample in the EncoderTracker, the logic largely relies on difference between consecutive time steps and thus we need atleast one value before rejecting anything based on a constraint. 
        volatile bool first_loop = true;

        // Sample vars.

        // Auto incrementing variable which indicates the time in microseconds taken since subsequent samples. Set to zero in the logic to reset it and start timing a new temporal sample segment.
        elapsedMicros micros_since_last_sample_ticker;
        // Cache variable to store the number of microseconds since last the sample cache retrieval.
        volatile uint32_t micros_since_last_sample;
        // Variable to indicate whether or not there is a new value in the sample buffer ready for retrieval.
        volatile bool sample_buffer_has_been_set = false;
        // The sample buffer cache.
        volatile uint32_t sample_buffer_cache[2] = {};
        // The index where we should extract the next sample from the sample buffer, the other index is being constantly updated as frequently as possible.
        volatile uint32_t sample_buffer_idx = 0;
        // The latest encoder value cached ready for retrieval.
        volatile uint32_t encoder_sample;

        // Issue counters.

        // The number of times that the wrong direction tolerance has been exceeded while sampling.
        uint32_t wrong_direction_ctr = 0.0;
        // The number of times that the skip steps tolerance has been exceeded while sampling.
        uint32_t skipped_steps_ctr = 0.0;

        // Tracking point vars.

        // The latest tracking point, aka the last known validated encoder measurent value.
        volatile uint32_t tracking_point = 0;

        // Value indicating whether or not the tracking point has been initalised to a valid sample value.
        volatile bool tracking_point_initalised = false;

        // The internal encoder instance.
        DigitalRotaryEncoderSPI encoder;

        // A boolean array describing the current configuration issues of the EncoderTracker, see the ConfigurationIssue enum deftype or ConfigurationIssueMessage char array for details.
        bool configuration_issues[6] = {false};

        /**
         * Method which actually takes the sample from the encoder, validates it according to the configured constraints it then either: triggers an internal fault and invokes the post_fault_logic method, or it updates the internal buffer values storing the sample for later retrieval and invokes the post_sample_logic method with the sample value.
         */
        void sample();

        /**
         * Internal method to change the fault state of the EncoderTracker and then immediately stops the sample timer.
         */
        void fault_now();
    };

#endif

}