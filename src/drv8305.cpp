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

  uint16_t command =  (DRV8305_CMD_WRITE << 15) 
                    | (DRV8305_GATE_DRIVE_CONTROL_REG << 11) 
                    | (DRV8305_GDC_COMM_OPTION_AF << 9)
                    | (DRV8305_GDC_PWM_MODE_3PWM << 7) 
                    | (DRV8305_GDC_DEAD_TIME_35ns << 4)
                    | (DRV8305_GDC_TBLANK_1us75 << 2) 
                    | (DRV8305_GDC_TVDS_3us5 << 0);

  uint8_t firstByte = (command >> 8);
  uint8_t secondByte = (command);

  int i = firstByte;

  byte resp1 = spi->transfer(firstByte); // B00111010
  byte resp2 = spi->transfer(secondByte); // B10000110
  
  digitalWrite(cs_pin, HIGH);

  Serial.println("Configure 3PWM Mode");
  Serial.print("Byte 0 : ");
  Serial.println(resp1, BIN);
  Serial.print("Byte 1 : ");
  Serial.println(resp2, BIN);
}

void DRV8305::configure6PWM_Mode() {
  digitalWrite(cs_pin, LOW);
  byte resp1 = spi->transfer(B00111010); // 0 - write, 0111 - 0x7 address
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
  byte resp1 = spi->transfer(B01001100); // 0 - write, 1001 - 0x9 address
  byte resp2 = spi->transfer(B10100000);
  digitalWrite(cs_pin, HIGH);

  Serial.println("Configure Amplier Clamping");
  Serial.print("Byte 0 : ");
  Serial.println(resp1, BIN);
  Serial.print("Byte 1 : ");
  Serial.println(resp2, BIN);
}