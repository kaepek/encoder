#include "digital_rotary_encoder.hpp"
#include "rotary_encoder_sample_validator.hpp"
#include "TeensyTimerTool.h"
#include <unordered_map>

using namespace TeensyTimerTool;

PeriodicTimer sample_timer(GPT1);

namespace kaepek
{
    RotaryEncoderSampleValidator::RotaryEncoderSampleValidator() {}

    RotaryEncoderSampleValidator::RotaryEncoderSampleValidator(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds)
    {
        this->encoder = encoder;
        this->skip_threshold = skip_threshold;
        this->skip_tolerance = skip_tolerance;
        this->sample_period_microseconds = sample_period_microseconds;
#ifdef KAEPEK_ENCODER_SKIP_DEFAULT_CONFIG
        this->set_skip_tolerance(DigitalRotaryEncoderSPI::skip_tolerance);
        this->set_skip_threshold(DigitalRotaryEncoderSPI::skip_threshold);
#endif
    }

    void RotaryEncoderSampleValidator::set_direction(Direction direction)
    {
        if (this->direction_enforcement == true)
        {
            this->direction = direction;
            this->direction_set = true;
        }
    }

    RotaryEncoderSampleValidator::Direction RotaryEncoderSampleValidator::get_direction()
    {
        if (this->direction_set == true)
        {
            return this->direction;
        }
        else
        {
            Serial.println("set_direction has to be invoked in order to retrieve a valid direction value");
            return Direction::UnSet;
        }
    }

    void RotaryEncoderSampleValidator::set_direction_enforcement(bool enabled)
    {
        this->direction_enforcement = enabled;
        this->direction_enforcement_set = true;
    }

    bool RotaryEncoderSampleValidator::get_direction_enforcement()
    {
        if (this->direction_enforcement_set == true)
        {
            return this->direction_enforcement;
        }
        else
        {
            Serial.println("get_direction_enforcement has to be invoked in order to retrieve a valid direction_enforcement value");
            return false;
        }
    }

    void RotaryEncoderSampleValidator::set_skip_tolerance(double tolerance)
    {
        if (tolerance > 0.0)
        {
            this->skip_tolerance = tolerance;
            this->skip_tolerance_set = true;
        }
    }

    double RotaryEncoderSampleValidator::get_skip_tolerance()
    {
        if (this->skip_tolerance_set == true)
        {
            return this->skip_tolerance;
        }
        else
        {
            Serial.println("set_skip_tolerance has to be invoked in order to retrieve a valid tolerance value");
            return 0.0;
        }
    }

    void RotaryEncoderSampleValidator::set_skip_threshold(uint32_t threshold)
    {
        if (threshold != 0)
        {
            this->skip_threshold = threshold;
            this->skip_threshold_set = true;
        }
    }

    uint32_t RotaryEncoderSampleValidator::get_skip_threshold()
    {
        if (this->skip_threshold_set == true)
        {
            return this->skip_threshold;
        }
        else
        {
            Serial.println("set_skip_threshold has to be invoked in order to retrieve a valid threshold value");
            return 0;
        }
    }

    void RotaryEncoderSampleValidator::set_wrong_way_tolerance(double tolerance)
    {
        if (this->direction_enforcement == true && tolerance > 0.0)
        {
            this->wrong_way_tolerance = tolerance;
            this->wrong_way_tolerance_set = true;
        }
    }

    double RotaryEncoderSampleValidator::get_wrong_direction_tolerance()
    {
        if (this->wrong_way_tolerance_set == true)
        {
            return this->wrong_way_tolerance;
        }
        else
        {
            Serial.println("set_wrong_way_tolerance has to be invoked in order to retrieve a valid tolerance value");
            return 0.0;
        }
    }

    void RotaryEncoderSampleValidator::set_wrong_way_threshold(uint32_t threshold)
    {
        if (this->direction_enforcement == true && threshold != 0)
        {
            this->wrong_way_threshold = threshold;
            this->wrong_way_threshold_set = true;
        }
    }

    uint32_t RotaryEncoderSampleValidator::get_wrong_way_threshold()
    {
        if (this->wrong_way_threshold_set == true)
        {
            return this->wrong_way_threshold;
        }
        else
        {
            Serial.println("set_wrong_way_threshold has to be invoked in order to retrieve a valid threshold value");
            return 0;
        }
    }

    void RotaryEncoderSampleValidator::setup()
    {
        this->encoder.setup();
        sample_timer.begin([this]
                           { this->sample(); },
                           this->sample_period_microseconds, false);
    }

    bool RotaryEncoderSampleValidator::start()
    {
        bool started_ok = true;

        if (this->skip_tolerance_set == false)
        {
            configuration_issues[ConfigurationIssue::SkipToleranceNotSet] = true;
            started_ok = false;
        }

        if (this->skip_threshold_set == false)
        {
            configuration_issues[ConfigurationIssue::SkipFaultThresholdNotSet] = true;
            started_ok = false;
        }

        if (this->direction_enforcement_set == false)
        {
            configuration_issues[ConfigurationIssue::DirectionEnforcementNotSet] = true;
            started_ok = false;
        }
        else
        {
            if (this->direction_enforcement == true)
            {
                if (this->direction_set == false)
                {
                    configuration_issues[ConfigurationIssue::DirectionNotSet] = true;
                    started_ok = false;
                }
                if (this->wrong_way_tolerance_set == false)
                {
                    configuration_issues[ConfigurationIssue::WrongWayToleranceNotSet] = true;
                    started_ok = false;
                }
                if (this->wrong_way_threshold_set == false)
                {
                    configuration_issues[ConfigurationIssue::WrongWayFaultThresholdNotSet] = true;
                    started_ok = false;
                }
            }
        }

        if (started_ok == true)
        {
            sample_timer.start();
            micros_since_last_sample_ticker = 0;
            started = true;
        }

        return started_ok;
    }

    void RotaryEncoderSampleValidator::print_configuration_issues()
    {
        for (uint32_t i = 0; i < 6; i++)
        {
            if (this->configuration_issues[i] == true)
            {
                Serial.println(RotaryEncoderSampleValidator::ConfigurationIssueMessage[i]);
            }
        }
        Serial.println("---------------------------------------");
    }

    void RotaryEncoderSampleValidator::stop()
    {
        sample_timer.stop();
        this->first_loop = true;
        this->started = false;
        this->sample_buffer_has_been_set = false;
        this->wrong_direction_ctr = 0;
        this->skipped_steps_ctr = 0;
    }

    bool RotaryEncoderSampleValidator::has_new_sample()
    {
        if (this->sample_buffer_has_been_set == false || this->fault == true)
        {
            return false;
        }
        cli();
        // Get latest encoder_value from value cache.
        uint32_t encoder_value = this->sample_buffer_cache[this->sample_buffer_idx];
        this->encoder_sample = encoder_value;
        this->sample_buffer_idx = (this->sample_buffer_idx + 1) % 2;
        this->sample_buffer_has_been_set = false;
        // Take time measurement since last cache reading.
        this->micros_since_last_sample = this->micros_since_last_sample_ticker;
        this->micros_since_last_sample_ticker = 0;
        sei();
        return true;
    }

    void RotaryEncoderSampleValidator::get_sample_and_elapsed_time(uint32_t &encoder_sample, uint32_t &micros_since_last_sample)
    {
        encoder_sample = this->encoder_sample;
        micros_since_last_sample = this->micros_since_last_sample;
    };

    void RotaryEncoderSampleValidator::post_sample_logic(uint32_t encoder_value)
    {
        Serial.println("Post sample logic run in base RotaryEncoderSampleValidator method");
    }

    void RotaryEncoderSampleValidator::post_fault_logic(Fault fault_code)
    {
        Serial.println("Post fault logic run in base RotaryEncoderSampleValidator method");
    }

    void RotaryEncoderSampleValidator::sample()
    {
        if (this->fault == true)
        {
            // Skip sampling if fault is set to true.
            return;
        }

        cli();
        uint32_t latest_encoder_value = this->encoder.read();
        sei();

        bool emit_a_new_value = false;

        if (this->first_loop == false)
        {
            double dx = this->encoder.get_reading_delta(this->tracking_point, latest_encoder_value);

            if (this->direction_enforcement == true)
            {
                // Skip and direction checking.

                double forward_tolerance, backward_tolerance;

                if (this->direction == Direction::Clockwise)
                { // cw
                    forward_tolerance = 1.0 * this->skip_tolerance;
                    backward_tolerance = -1.0 * this->wrong_way_tolerance;
                }
                else
                { // ccw
                    forward_tolerance = -1.0 * this->skip_tolerance;
                    backward_tolerance = 1.0 * this->wrong_way_tolerance;
                }

                if (this->direction == Direction::Clockwise) // +ve cw
                {
                    if (dx > (forward_tolerance * (double)(this->skipped_steps_ctr + 1)))
                    {
                        // We have skipped forwards.
                        this->skipped_steps_ctr++;
                    }
                    else if (dx < 0.0)
                    {
                        // Going backwards, backward_tolerance -ve.

                        if (dx < backward_tolerance)
                        {
                            // Yes wrong way tolerance exceeded indicating a detection.
                            this->wrong_direction_ctr++;
                        }
                        else
                        {
                            // Within tolerance
                            this->skipped_steps_ctr = 0;
                            this->wrong_direction_ctr = 0;
                            this->sample_buffer_cache[this->sample_buffer_idx] = latest_encoder_value;
                            this->sample_buffer_has_been_set = true;
                            // Dont move tracking point.
                            emit_a_new_value = true;
                        }
                    }
                    else
                    {
                        // Accept.
                        this->sample_buffer_cache[this->sample_buffer_idx] = latest_encoder_value;
                        this->tracking_point = latest_encoder_value;
                        this->skipped_steps_ctr = 0;
                        this->wrong_direction_ctr = 0;
                        this->sample_buffer_has_been_set = true;
                        emit_a_new_value = true;
                    }
                }
                else // -ve ccw
                {
                    if (dx < (forward_tolerance * (double)(this->skipped_steps_ctr + 1)))
                    {
                        // We have skipped forwards.
                        this->skipped_steps_ctr++;
                    }
                    else if (dx > 0.0)
                    {
                        // Going backwards, backward_tolerance +ve
                        if (dx > backward_tolerance)
                        {
                            this->wrong_direction_ctr++;
                        }
                        else
                        {
                            // Within tolerance.
                            this->skipped_steps_ctr = 0;
                            this->wrong_direction_ctr = 0;
                            this->sample_buffer_cache[this->sample_buffer_idx] = latest_encoder_value;
                            this->sample_buffer_has_been_set = true;
                            // Dont move tracking point.
                            emit_a_new_value = true;
                        }
                    }
                    else
                    {
                        // Accept.
                        this->sample_buffer_cache[this->sample_buffer_idx] = latest_encoder_value;
                        this->tracking_point = latest_encoder_value;
                        this->skipped_steps_ctr = 0;
                        this->wrong_direction_ctr = 0;
                        this->sample_buffer_has_been_set = true;
                        emit_a_new_value = true;
                    }
                }
            }
            else
            {
                // Just skip checking.
                if (dx > (this->skip_tolerance * (double)(this->skipped_steps_ctr + 1)))
                {
                    // We have skipped forwards.
                    this->skipped_steps_ctr++;
                }
                else if (dx < (-1.0 * this->skip_tolerance * (double)(this->skipped_steps_ctr + 1)))
                {
                    // We have skipped backwards.
                    this->skipped_steps_ctr++;
                }
                else
                {
                    // Accept.
                    this->sample_buffer_cache[this->sample_buffer_idx] = latest_encoder_value;
                    this->tracking_point = latest_encoder_value;
                    this->skipped_steps_ctr = 0;
                    this->wrong_direction_ctr = 0;
                    this->sample_buffer_has_been_set = true;
                    emit_a_new_value = true;
                }
            }

            // Skip threshold exceeded.
            if (this->skipped_steps_ctr > this->skip_threshold)
            {
                // Fault internally.
                this->fault_now();
                // Invoke additional fault logic.
                this->post_fault_logic(Fault::SkippedSteps);
                return;
            }
            // Skip wrong direction threshold exceeded.
            else if (this->wrong_direction_ctr > this->wrong_way_threshold)
            {
                // Fault internally.
                this->fault_now();
                // Invoke additional fault logic.
                this->post_fault_logic(Fault::WrongDirection);
                return;
            }

            // Invoke additional logic here. If value is released from buffer, e.g. commute logic.
            if (emit_a_new_value == true)
            {
                cli();
                // Invoke external logic.
                this->post_sample_logic(latest_encoder_value); // Careful logic extension must complete within the period time. Also if this takes too long, then the main loop will suffer less execution time.
                sei();
            }
        }
        else // First value measurement, we must accept this, but take <initial_measurements> measurements to avoid bad values and take the most common result.
        {
            cli();
            // take <initial_measurements> measurements
            int initial_readings[initial_measurements] = {};
            for (int i = 0; i < initial_measurements; i++)
            {
                delayMicroseconds(100);
                initial_readings[i] = this->encoder.read();
            }
            // find the most frequent element
            std::unordered_map<int, int> frequency_map;
            int max_count = 0;
            int most_frequent_reading = initial_readings[0];
            for (int i = 0; i < initial_measurements; i++)
            {
                frequency_map[initial_readings[i]]++;
                if (frequency_map[initial_readings[i]] > max_count)
                {
                    max_count = frequency_map[initial_readings[i]];
                    most_frequent_reading = initial_readings[i];
                }
            }
            // apply this initial reading
            this->sample_buffer_idx = 0;
            this->sample_buffer_cache[this->sample_buffer_idx] = most_frequent_reading;
            this->tracking_point = most_frequent_reading;
            this->first_loop = false;
            sei();
        }
    }

    void RotaryEncoderSampleValidator::fault_now()
    {
        this->fault = true;
        this->stop();
    }

    void RotaryEncoderSampleValidator::reset()
    {
        this->stop();
        this->fault = false;
    }

}