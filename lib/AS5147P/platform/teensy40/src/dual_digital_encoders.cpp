#include "generic/dual_digital_encoders.hpp"

namespace kaepek
{
    const int DIGITAL_ENCODER_AS5147P_BUFFER_LENGTH = 16;
    const uint32_t DIGITAL_ENCODER_AS5147P_BUFFER_MAX_INDEX = 15;
    const uint32_t DIGITAL_ENCODER_AS5147P_BUFFER_VALUE_MASK = 0x3FFF;

    const uint32_t DigitalEncoderSPI::encoder_divisions = 16384;
    const uint32_t DigitalEncoderSPI::encoder_divisions_over_2 = 16384 / 2;

    DualDigitalEncodersSPI::DualDigitalEncodersSPI(DigitalEncoderSPI enc1, DigitalEncoderSPI enc2)
    {
        this->enc1 = enc1;
        this->enc2 = enc2;
        this->pins_enc1 = enc1.getPins();
        this->pins_enc2 = enc2.getPins();
    }

    DualDigitalEncodersSPI::DualDigitalEncodersSPI()
    {
    }

    void DualDigitalEncodersSPI::setup()
    {
        this->enc1.setup();
        this->enc2.setup();
    };

    DualEncodersSensorValues DualDigitalEncodersSPI::read()
    {
        // Define slave bit buffer.
        bool enc1_miso_buffer_bit = 0;
        bool enc2_miso_buffer_bit = 0;

        uint32_t enc_1_value = 0;
        enc_1_value <<= 16;

        uint32_t enc_2_value = 0;
        enc_2_value <<= 16;

        // Set CSN low (start a frame) -------------------------------------------
        digitalWriteFast(this->pins_enc1.csn, LOW);
        digitalWriteFast(this->pins_enc2.csn, LOW);

        // DIGITAL_ENCODER_AS5147P_BUFFER_LENGTH is 16.

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value <<= 1;
        enc_2_value <<= 1;
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 1;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 1;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 1;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 1;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 1;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 1;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 2;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 1;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 1;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 1;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 1;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 1;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);

        // ---------
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_1_value <<= 1;
        enc_2_value |= enc2_miso_buffer_bit;
        enc_2_value <<= 1;
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
        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);
        enc1_miso_buffer_bit = digitalReadFast(this->pins_enc1.miso);
        enc2_miso_buffer_bit = digitalReadFast(this->pins_enc2.miso);
        enc_1_value |= enc1_miso_buffer_bit;
        enc_2_value |= enc2_miso_buffer_bit;

        // These last 2 bits are not important for the encode measurment value, in this faster reading mode which ignores the parity.
        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);

        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);

        digitalWriteFast(this->pins_enc1.sck, HIGH);
        digitalWriteFast(this->pins_enc2.sck, HIGH);

        enc_1_value <<= 2;
        enc_2_value <<= 2;

        digitalWriteFast(this->pins_enc1.sck, LOW);
        digitalWriteFast(this->pins_enc2.sck, LOW);

        // Set CSN high (end frame) -----------------------------------------------
        digitalWriteFast(this->pins_enc1.csn, HIGH);
        digitalWriteFast(this->pins_enc2.csn, HIGH);

        // Extract the 14 bits from the 16 bit buffer; which represents the encoder measurement value.
        // ENCODER_BUFFER_enc_1_value_MASK is 14 1's

        enc_1_value = (enc_1_value & DIGITAL_ENCODER_AS5147P_BUFFER_VALUE_MASK);
        enc_2_value = (enc_2_value & DIGITAL_ENCODER_AS5147P_BUFFER_VALUE_MASK);

        DualEncodersSensorValues output = DualEncodersSensorValues();
        output.enc1_val = enc_1_value;
        output.enc2_val = enc_2_value;

        return output;
    }

    void DualDigitalEncodersSPI::read(DualEncodersSensorValues &sensorValues)
    {
        sensorValues = read();
    }

}