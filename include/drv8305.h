#include <Arduino.h>
#include <SimpleFOC.h>

class DRV8305 {
private:
  uint8_t cs_pin;
  uint8_t ena_gate_pin;
  SPIClass *spi;
public:
  DRV8305(SPIClass *spi, int cs_pin, int ena_gate_pin);
  
  void Init();

  // Set to three PWM inputs mode to 3PWM
  void configure3PWM_Mode();

  // Set to three PWM inputs mode to 6PWM
  void configure6PWM_Mode();

  // Clamp sense amplifier output to 3.3V
  void configureAmplierClamping();
};