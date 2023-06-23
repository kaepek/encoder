#include "lib/digital_encoder.cpp"
#include "lib/generic/encoder_sample_validator.cpp"

// An example extension of the EncoderSampleValidator class.
namespace kaepek
{
  class Demo : public EncoderSampleValidator
  {
  public:
    // Variable to hold the latest sample of the encoders measurement value.
    uint32_t latest_encoder_value = 0;

    // Default constuctor.
    Demo() : EncoderSampleValidator()
    {
    }

    // Constructor with parameters.
    Demo(DigitalEncoderSPI encoder, float sample_period_microseconds) : EncoderSampleValidator(encoder, sample_period_microseconds)
    {
    }

    void post_sample_logic(uint32_t encoder_value) override
    {
      // Update the latest_encoder_value cache with the latest validated encoder value.
      latest_encoder_value = encoder_value;
    }

    void post_fault_logic(EncoderSampleValidator::Fault fault_code) override
    {
      // Do something fault specific based on the <fault_code>.
      if (fault_code == EncoderSampleValidator::Fault::SkippedSteps)
      {
        // Print fault notification.
        Serial.println("FAULTED: Skipped steps");
      }
      else if (fault_code == EncoderSampleValidator::Fault::WrongDirection)
      {
        // Print fault notification.
        Serial.println("FAULTED: Wrong direction");
      }
    }
  };
}

// Define encoder pin config struct.
kaepek::DigitalEncoderPinsSPI enc_pins = kaepek::DigitalEncoderPinsSPI();
// Define the encoder.
kaepek::DigitalEncoderSPI enc;
// Define the encoder sampler.
kaepek::Demo sampler;
// Define bool for knowing if the sampler started is a good state.
bool started_ok = false;

void setup()
{
  // Setup the encoder pin configuration.
  enc_pins.csn = 10;
  enc_pins.miso = 12;
  enc_pins.mosi = 11;
  enc_pins.sck = 22;
  
  // Initalise the encoder with giving it the pin configuration.
  enc = kaepek::DigitalEncoderSPI(enc_pins);

  // Initalise the encoder sampler.
  sampler = kaepek::Demo(enc, 2.0); // 2us (micro) sample period

  // Allow skipping ahead a maximum value of 4.0, in terms of the read encoder value measurement, before a skip is detected.
  sampler.set_skip_tolerance(4.0);
  // Only allow skipping ahead twice before faulting.
  sampler.set_skip_threshold(2);

  // To enable direction enforcement, invoke the sampler set_direction_enforcement method with a true argument.
  sampler.set_direction_enforcement(true);
  // Set direction validation to be Clockwise (otherwise you could choose 'CounterClockwise').
  sampler.set_direction(kaepek::EncoderSampleValidator::Direction::Clockwise);
  // Specify the maximum encoder value change allowed in the wrong direction, before wrong direction motion is detected.
  sampler.set_wrong_way_tolerance((double)kaepek::DigitalEncoderSPI::encoder_divisions / (14.0)); // For a 14 pole motor.
  // Allow two detections of motion in wrong direction before faulting.
  sampler.set_wrong_way_threshold(2);

  // To disable direction enforcement, invoke the sampler set_direction_enforcement method with a false argument.
  // sampler.set_direction_enforcement(false); // Nothing else, otherwise if set_direction, set_wrong_way_tolerance and/or set_wrong_way_threshold are invoked, they will be ignored silently.

  // Run setup procedure of the sampler. Note this will invoke the encoder's setup method and therefore it is unnecessary to do it explicitly on the encoder instance.
  sampler.setup();
  // Start sampling.
  started_ok = sampler.start();
}

void loop()
{
  if (started_ok == true)
  {
    // Check the encoder has a new sample.
    if (sampler.has_new_sample() == true)
    {
      // Define variables to store the sampled encoder value and the number of elapsed microseconds since the last samples retrieval.
      uint32_t encoder_value;
      uint32_t elapsed_micros_since_last_sample;
      // Fetch the stored values from the buffer.
      sampler.get_sample_and_elapsed_time(encoder_value, elapsed_micros_since_last_sample);
      // Print the validated encoder value out via the serial port.
      Serial.print("Current encoder value:\t");
      Serial.print(sampler.latest_encoder_value);
      Serial.print("\n");
      // Pause as to not spam the serial port too much.
      delayMicroseconds(100);
    }
  }
  else {
    // If the sampler did not start in a good state, then print the configuration issues out via the serial port, by invoking the print_configuration_issues method of the sampler.
    sampler.print_configuration_issues();
    delayMicroseconds(10'000'000);
  }
}
