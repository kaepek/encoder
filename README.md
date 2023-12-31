# KAEPEK-ENCODER V2.1.1

Drivers for microcontrollers enabling them to read encoders and supporting tooling to aid using them for real world applications.

# Rotary encoders:

## Digital rotary encoders:

### Supported digital rotary encoders:

- [AS5147P](./lib/rotary/AS5147P/README.md)

### DigitalRotaryEncoderSPI/DualDigitalRotaryEncodersSPI usage example:

A digital encoder with an SPI interface can be read by configuring the ```DigitalEncoderPinsSPI``` struct and initalising the ```DigitalRotaryEncoderSPI``` class with the pins struct argument. Moreover one can choose
to initalise the ```DualDigitalRotaryEncodersSPI``` class with two ```DigitalRotaryEncoderSPI``` instances, this enables reading from two encoders simultaneously maximising the sampling frequency of the combined system. 

```
#include "lib/rotary/{ENCODER_NAME}/platform/{MICROCONTROLLER}/digital_rotary_encoder.cpp"
#include "lib/rotary/{ENCODER_NAME}/platform/{MICROCONTROLLER}/dual_digital_rotary_encoders.cpp"

kaepek::DigitalEncoderPinsSPI enc1_pins = kaepek::DigitalEncoderPinsSPI();
kaepek::DigitalEncoderPinsSPI enc2_pins = kaepek::DigitalEncoderPinsSPI();
kaepek::DigitalRotaryEncoderSPI enc1;
kaepek::DigitalRotaryEncoderSPI enc2;
kaepek::DualDigitalRotaryEncodersSPI dualenc;

void setup() {
  // Set encoder pins.
  enc_pins1.csn = 10;
  enc_pins1.miso = 12;
  enc_pins1.mosi = 11;
  enc_pins1.sck = 22;
  enc_pins2.csn = 14;
  enc_pins2.miso = 15;
  enc_pins2.mosi = 16;
  enc_pins2.sck = 17;

  // Init encoder classes.
  enc1 = kaepek::DigitalRotaryEncoderSPI(enc1_pins);
  enc2 = kaepek::DigitalRotaryEncoderSPI(enc2_pins);
  dualenc = kaepek::DualDigitalRotaryEncodersSPI(enc1, enc2);
}

void loop() {
    // Read and print enc1.
    Serial.println(enc1.read());
    // Read and print enc2.
    Serial.println(enc2.read());
    // Read enc1 and enc2 at the same time.
    kaepek::DualEncodersSensorValues values = dualenc.read();
    // Print the dual read values.
    Serial.println(values.enc1_val);
    Serial.println(values.enc2_val);
}

```

### Rotary encoder sample validator:

Encoders are physical devices which have an intrinsic error and which may fail (or the connecting wires may fail). Thus a class ```RotaryEncoderSampleValidator``` is provided to aid
fault detection from implausable encoder readings, and to then invoke additional behaviour; in order to deal with the situation. Moreover for certain applications, one may wish to additionally validate that the encoder is only moving in a single specified direction (clockwise/anti-clockwise), moving against the configured direction could indicate a fault state, where additional logic should be invoked; in order to deal that situation. For instance a record deck for Hi-Fi applications could use a needle, that could be broken by motion of the deck in the wrong direction (which may occur by incorrect motor phase wiring, e.g. by accidentally swapping phases A & C, for a 3 phase system) and thus when this fault is detected, the motor should be shutoff, to prevent damage to the needle.

One may inherit from the class ```RotaryEncoderSampleValidator``` and extended it such that the supplied extended behaviour can automatically be invoked, when the class instance detects a faulty state. Please note that this class currently may only be used with the ```DigitalRotaryEncoderSPI``` class, Kaepek aims to support the ```DualDigitalRotaryEncodersSPI``` class with a ```DualEncodersSampleValidator``` class at somepoint in the future.

### Rotary encoder sample validator usage example:

```
#include "lib/rotary/{ENCODER_NAME}/platform/{MICROCONTROLLER}/digital_rotary_encoder.cpp"
#include "lib/rotary/generic/rotary_encoder_sample_validator.cpp"

/**
* Example extension to the RotaryEncoderSampleValidator class:
*
* An electronic speed controller could be implemented: post_sample_logic could do PWM switching
* based on encoder's validated position and post_fault_logic could deal with faults by switching off the PWM pins.
*/
namespace kaepek
{
  class DemoESC : public RotaryEncoderSampleValidator
  {
  public:
    // Default constuctor.
    DemoESC() : RotaryEncoderSampleValidator()
    {
    }

    // Constructor with parameters.
    DemoESC(DigitalRotaryEncoderSPI encoder, float sample_period_microseconds) : RotaryEncoderSampleValidator(encoder, sample_period_microseconds)
    {
    }

    void post_sample_logic(uint32_t encoder_value)
    {
      // Set PWM pin state for this commutation, based upon the current validated encoder position <encoder_value>.
    }

    void post_fault_logic(RotaryEncoderSampleValidator::Fault fault_code)
    {
      // Turn off motor by setting PWM pins off.

      // Do something fault specific based on the <fault_code>.
      if (fault_code == RotaryEncoderSampleValidator::Fault::SkippedSteps)
      {
      }
      else if (fault_code == RotaryEncoderSampleValidator::Fault::WrongDirection)
      {
      }
    }
  };
}

// Define encoder pin config struct.
kaepek::DigitalEncoderPinsSPI enc_pins = kaepek::DigitalEncoderPinsSPI();
// Define the encoder.
kaepek::DigitalRotaryEncoderSPI enc;
// Define the encoder sampler.
kaepek::DemoESC sampler;
// Define bool for knowing if the sampler started is a good state.
bool started_ok;

void setup()
{
  // Setup the encoder pin configuration.
  enc_pins.csn = 10;
  enc_pins.miso = 12;
  enc_pins.mosi = 11;
  enc_pins.sck = 22;
  
  // Initalise the encoder with giving it the pin configuration.
  enc = kaepek::DigitalRotaryEncoderSPI(enc_pins);

  // Initalise the encoder sampler.
  sampler = kaepek::DemoESC(enc, 2.0); // 2us (micro) sample period

  // Allow skipping ahead a maximum value of 4.0, in terms of the read encoder value measurement, before a skip is detected.
  sampler.set_skip_tolerance(4.0);
  // Only allow skipping ahead twice before faulting.
  sampler.set_skip_threshold(2);

  // To enable direction enforcement, invoke the sampler set_direction_enforcement method with a true argument.
  sampler.set_direction_enforcement(true);
  // Set direction validation to be Clockwise (otherwise you could choose 'CounterClockwise').
  sampler.set_direction(kaepek::RotaryEncoderSampleValidator::Direction::Clockwise);
  // Specify the maximum encoder value change allowed in the wrong direction, before wrong direction motion is detected.
  sampler.set_wrong_way_tolerance((double)kaepek::DigitalRotaryEncoderSPI::encoder_divisions / (14.0)); // For a 14 pole motor.
  // Allow two detections of motion in wrong direction before faulting.
  sampler.set_wrong_way_threshold(2);

  // To disable direction enforcement, invoke the sampler set_direction_enforcement method with a false argument.
  // sampler.set_direction_enforcement(false); // Nothing else, otherwise if set_direction, set_wrong_way_tolerance and/or set_wrong_way_threshold are invoked, they will be ignored silently.

  // Run setup procedure of the sampler. Note this will invoke the encoder's setup method and therefore it is unnecessary to do it explicitly on the encoder instance.
  sampler.setup();
  // Start sampling.
  started_ok = sampler.start();
}

// Main loop runs as frequently as it can.
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
      // Do something else like speed computations etc.
    }
  }
  else {
    // If the sampler did not start in a good state, then print the configuration issues out via the serial port, by invoking the print_configuration_issues method of the sampler.
    sampler.print_configuration_issues();
    delayMicroseconds(10'000'000);
  }

}
```
### Useful links
- [Rotary encoder sensor overview1](https://www.electronicproducts.com/absolute-position-sensing-the-key-to-better-brushless-dc-motor-control/)
- [Rotary encoder sensor overview2](https://www.seeedstudio.com/blog/2020/01/19/rotary-encoders-how-it-works-how-to-use-with-arduino/)

# Linear encoders:

- Todo

# Dependancies for any EncoderSampleValidator:

- [TeensyTimerTool](https://github.com/luni64/TeensyTimerTool/blob/master/LICENSE)

# How to prepare the Teensy40 platform in order to use any EncoderSampleValidator:
- Install [Arduino IDE v1.8.19](https://www.arduino.cc/en/software)
- Install [Teensyduino v2.1.0](https://www.pjrc.com/teensy/teensyduino.html)
- Install [TeensyTimerTool library](https://github.com/luni64/TeensyTimerTool) by opening up Arduino IDE click the "Sketch" menu item, go down to "Include Library" and click "Manage Libraries...". Enter in the filter you search the string "TeensyTimerTool", select the correct version "Version 1.3.0" and click the install button.

# General dependancies:

- [Arduino.h](https://github.com/arduino/ArduinoCore-avr)
- [imxrt.h](https://github.com/PaulStoffregen/cores/tree/master)

