#include <Arduino.h>
#include <SimpleFOC.h>

#define DRV8305_CMD_WRITE 0
#define DRV8305_CMD_READ 1

#define DRV8305_GATE_DRIVE_CONTROL_REG 0x7
#define DRV8305_IC_OPERATION_REG 0x9

#define DRV8305_GDC_COMM_OPTION_AF 0x1

#define DRV8305_GDC_PWM_MODE_6PWM 0x0
#define DRV8305_GDC_PWM_MODE_3PWM 0x1

#define DRV8305_GDC_DEAD_TIME_35ns 0x0
#define DRV8305_GDC_DEAD_TIME_52ns 0x1
#define DRV8305_GDC_DEAD_TIME_88ns 0x2

#define DRV8305_GDC_TBLANK_1us75 0x1
#define DRV8305_GDC_TVDS_3us5 0x2

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