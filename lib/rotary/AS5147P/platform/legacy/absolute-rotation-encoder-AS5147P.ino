#include "src/main.cpp"

void setup() {
  as5147p_setup();
}

uint16_t value;

/*
 #define csn 10
#define sck 22
#define miso 12
#define mosi 11
 */
void loop() {
  // put your main code here, to run repeatedly:
  auto parity = as5147p_get_sensor_value(value);
  Serial.print(value);
  Serial.print("\t");
  Serial.print(parity);
  Serial.print("\n");
  // delayMicroseconds(1000); // 1000hz
  delayMicroseconds(100); // 10000hz
}
