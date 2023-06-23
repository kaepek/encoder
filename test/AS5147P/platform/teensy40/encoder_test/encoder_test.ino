#include "lib/digital_encoder.cpp"

// Define encoder pin config struct.
kaepek::DigitalEncoderPinsSPI enc_pins = kaepek::DigitalEncoderPinsSPI();
// Define the encoder.
kaepek::DigitalEncoderSPI enc;

void setup()
{
  // Setup the encoder pin configuration.
  enc_pins.csn = 10;
  enc_pins.miso = 12;
  enc_pins.mosi = 11;
  enc_pins.sck = 22;
  
  // Initalise the encoder with giving it the pin configuration.
  enc = kaepek::DigitalEncoderSPI(enc_pins);

  // setup encoder so that its ready to read
  enc.setup();
}

void loop()
{
  delayMicroseconds(100);
  Serial.print(enc.read());
  Serial.print("\n");
}
