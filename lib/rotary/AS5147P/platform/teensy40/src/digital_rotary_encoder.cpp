#include "generic/digital_rotary_encoder.hpp"

namespace kaepek
{

    const int DIGITAL_ENCODER_AS5147P_BUFFER_LENGTH = 16;
    const uint32_t DIGITAL_ENCODER_AS5147P_BUFFER_MAX_INDEX = 15;
    const uint32_t DIGITAL_ENCODER_AS5147P_BUFFER_VALUE_MASK  = 0x3FFF;

    const uint32_t DigitalRotaryEncoderSPI::encoder_divisions = 16384;
    const uint32_t DigitalRotaryEncoderSPI::encoder_divisions_over_2 = 16384 / 2;

#ifndef KAEPEK_ENCODER_SKIP_DEFAULT_CONFIG
#define KAEPEK_ENCODER_SKIP_DEFAULT_CONFIG
    const double DigitalRotaryEncoderSPI::skip_tolerance = 4.0;
    const uint32_t DigitalRotaryEncoderSPI::skip_threshold = 2;
#endif

    double DigitalRotaryEncoderSPI::get_reading_delta(double old_value, double new_value)
    {
        // Take difference.
        double delta = new_value - old_value;
        // Apply modulus.
        delta = delta - DigitalRotaryEncoderSPI::encoder_divisions * floor(delta / DigitalRotaryEncoderSPI::encoder_divisions);
        if (delta > DigitalRotaryEncoderSPI::encoder_divisions_over_2)
        {
            return -(DigitalRotaryEncoderSPI::encoder_divisions - delta);
        }
        else
        {
            return delta;
        }
    }

    DigitalRotaryEncoderSPI::DigitalRotaryEncoderSPI()
    {
    }

    DigitalRotaryEncoderSPI::DigitalRotaryEncoderSPI(DigitalEncoderPinsSPI pins)
    {
        this->pins = pins;
    }

    DigitalEncoderPinsSPI DigitalRotaryEncoderSPI::getPins()
    {
        return this->pins;
    }

    void DigitalRotaryEncoderSPI::setup()
    {
        // Set pin modes.
        pinMode(this->pins.csn, OUTPUT);
        pinMode(this->pins.mosi, OUTPUT);
        pinMode(this->pins.miso, INPUT);
        pinMode(this->pins.sck, OUTPUT);
        // Set read angle command bits.
        digitalWrite(this->pins.mosi, HIGH); // This can be left high, as the command is all ones.
    };

    void DigitalRotaryEncoderSPI::read(uint32_t &value, bool &parity)
    {
        // Define parity check bit and slave bit buffer.
        bool miso_buffer_bit, parity_bit_check = 0;

        value = 0;

        // Set CSN low (start a frame) -------------------------------------------
        digitalWriteFast(this->pins.csn, LOW);

        // For each bit in buffer.
        for (int buffer_ctr = 0; buffer_ctr < DIGITAL_ENCODER_AS5147P_BUFFER_LENGTH; buffer_ctr++)
        {
            // Set clock high.
            digitalWriteFast(this->pins.sck, HIGH);
            value <<= 1; // Bit shift old read value one to left, so we can collect a new bit from MISO.
            delayNanoseconds(100);
            // Set clock low.
            digitalWriteFast(this->pins.sck, LOW);
            // Now SCK has been pulsed, the slave will have written a bit to MISO.
            // Read one bit.
            miso_buffer_bit = digitalReadFast(this->pins.miso);
            // Append the latest bit to value.
            value |= miso_buffer_bit;

            // Sum parity.
            if (buffer_ctr > 0)
            {
                parity_bit_check += miso_buffer_bit;
            }
        }
        delayNanoseconds(10);

        // Set CSN high (end frame) -----------------------------------------------
        digitalWriteFast(this->pins.csn, HIGH);

        // Final parity_bit_check check against first bit.
        bool parity_check_result = (parity_bit_check & 1) != (value >> DIGITAL_ENCODER_AS5147P_BUFFER_MAX_INDEX);

        // Extract the 14 bits from the 16 bit buffer; which represents the encoder measurement value.
        value = (value & DIGITAL_ENCODER_AS5147P_BUFFER_VALUE_MASK);

        parity = parity_check_result;
    }

    uint32_t DigitalRotaryEncoderSPI::read()
    {
        // Define parity check bit and slave bit buffer.
        bool miso_buffer_bit = 0; 

        uint32_t value = 0;
        value <<= 16;

        // Set CSN low (start a frame) -------------------------------------------
        digitalWriteFast(this->pins.csn, LOW);

        // DIGITAL_ENCODER_AS5147P_BUFFER_LENGTH is 16.

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);

        // ---------
        digitalWriteFast(this->pins.sck, HIGH);
        value |= miso_buffer_bit;
        value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins.sck, LOW);
        miso_buffer_bit = digitalReadFast(this->pins.miso);
        value |= miso_buffer_bit;

        // These last 2 bits are not important for the encode measurment value, in this faster reading mode which ignores the parity.
        digitalWriteFast(this->pins.sck, HIGH);
        digitalWriteFast(this->pins.sck, LOW);
        digitalWriteFast(this->pins.sck, HIGH);
        value <<= 2;
        digitalWriteFast(this->pins.sck, LOW);

        // Set CSN high (end frame) -----------------------------------------------
        digitalWriteFast(this->pins.csn, HIGH);

        // Extract the 14 bits from the 16 bit buffer; which represents the encoder measurement value.
        // DIGITAL_ENCODER_AS5147P_BUFFER_VALUE_MASK is 14 1's.

        value = (value & DIGITAL_ENCODER_AS5147P_BUFFER_VALUE_MASK);

        return value;
    }

    void DigitalRotaryEncoderSPI::read(uint32_t &value)
    {
        value = read();
    }
}