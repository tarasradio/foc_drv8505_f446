#include <drv8305.h>

DRV8305::DRV8305(SPIClass *spi, int cs_pin, int ena_gate_pin) {
  this->spi = spi;
  this->cs_pin = cs_pin;
  this->ena_gate_pin = ena_gate_pin;
}

void DRV8305::Init() {
  Serial.println("DRV8305: init");

  pinMode(ena_gate_pin, OUTPUT);
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);

  configure6PWM_Mode();
  configureAmplierClamping();

  _delay(500);

  Serial.println("DRV8305: enGate Enabled");

  digitalWrite(ena_gate_pin, HIGH);
}

void DRV8305::configure3PWM_Mode() {
  digitalWrite(cs_pin, LOW);
  byte resp1 = spi->transfer(B00111010); // 0 - write, 0111 - 0x7 address, 0 1 01 000 01 10
  byte resp2 = spi->transfer(B10000110);
  digitalWrite(cs_pin, HIGH);

  Serial.println("Configure 3PWM Mode");
  Serial.print("Byte 0 : ");
  Serial.println(resp1, BIN);
  Serial.print("Byte 1 : ");
  Serial.println(resp2, BIN);
}

void DRV8305::configure6PWM_Mode() {
  digitalWrite(cs_pin, LOW);
  byte resp1 = spi->transfer(B00111010);
  byte resp2 = spi->transfer(B0000110);
  digitalWrite(cs_pin, HIGH);

  Serial.println("Configure 6PWM Mode");
  Serial.print("Byte 0 : ");
  Serial.println(resp1, BIN);
  Serial.print("Byte 1 : ");
  Serial.println(resp2, BIN);
}

void DRV8305::configureAmplierClamping() {
  digitalWrite(cs_pin, LOW);
  byte resp1 = spi->transfer(B01001100);
  byte resp2 = spi->transfer(B10100000);
  digitalWrite(cs_pin, HIGH);

  Serial.println("Configure Amplier Clamping");
  Serial.print("Byte 0 : ");
  Serial.println(resp1, BIN);
  Serial.print("Byte 1 : ");
  Serial.println(resp2, BIN);
}