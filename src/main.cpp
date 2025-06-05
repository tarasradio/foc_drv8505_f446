#include <Arduino.h>
#include <SimpleFOC.h>
#include <GyverFilters.h>

#include "STM32_CAN.h"
#include "definitions.h"
#include <drv8305.h>
#include <move_controller.h>

STM32_CAN Can1( CAN1, DEF );  //Use PA11/12 pins for CAN1.

static CAN_message_t CAN_TX_msg;
static CAN_message_t CAN_RX_msg;

#define MOTOR_PP 10 // pole pair number (14 for Hypershell, 10 for DNSYS)

// SPI2
#define MOSI_ENC_A PC3
#define MISO_ENC_A PC2
#define CLK_ENC_A PB10
#define CS_ENC_A PB12

// SPI3
#define MOSI_ENC_B PC12
#define MISO_ENC_B PC11
#define CLK_ENC_B PC10
#define CS_ENC_B PD2

// SPI1
#define MOSI_DRV PA7
#define MISO_DRV PA6
#define CLK_DRV PA5
#define CS_DRV PA4

#define INH_A PA8 // TIM1_CH1
#define INH_B PA9 // TIM1_CH2
#define INH_C PA10 // TIM1_CH3

#define INL_A PB13 // TIM1-CH1N
#define INL_B PB14 // TIM1-CH2N
#define INL_C PB15 // TIM1-CH3N

#define ENA_GATE PB4

#define SENSE_A PC0
#define SENSE_B PC1
#define SENSE_C PB0

SPIClass spiConnectionEncoderA(MOSI_ENC_A, MISO_ENC_A, CLK_ENC_A);
SPIClass spiConnectionEncoderB(MOSI_ENC_B, MISO_ENC_B, CLK_ENC_B);

MagneticSensorSPIConfig_s AS5047P_SPI_Config_A{
  .spi_mode = SPI_MODE1,
  .clock_speed = 10000000,
  .bit_resolution = 14,
  .angle_register = 0x3FFF,
  .data_start_bit = 13,
  .command_rw_bit = 14,
  .command_parity_bit = 15};

MagneticSensorSPIConfig_s AS5047P_SPI_Config_B{
  .spi_mode = SPI_MODE1,
  .clock_speed = 10000000,
  .bit_resolution = 14,
  .angle_register = 0x3FFF,
  .data_start_bit = 13,
  .command_rw_bit = 14,
  .command_parity_bit = 15};

MagneticSensorSPI encoderA = MagneticSensorSPI(AS5047P_SPI_Config_A, CS_ENC_A);
MagneticSensorSPI encoderB = MagneticSensorSPI(AS5047P_SPI_Config_B, CS_ENC_B);

SPIClass spiConnectionDRV(MOSI_DRV, MISO_DRV, CLK_DRV);

DRV8305 drv8305(&spiConnectionDRV, CS_DRV, ENA_GATE);

BLDCMotor motor = BLDCMotor(MOTOR_PP);

BLDCDriver6PWM driver = BLDCDriver6PWM(INH_A, INL_A, INH_B, INL_B, INH_C, INL_C);

LowsideCurrentSense current_sense = LowsideCurrentSense(0.007f, 12.22f, SENSE_A, SENSE_B, SENSE_C);

MoveController moveController(&motor);

void configureMotor() {
  // control loop type and torque mode
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::velocity;
  
  // controller configuration 
  // default parameters in defaults.h

  // controller configuration based on the control type 
  // velocity PID controller parameters
  // default P=0.5 I = 10 D =0
  motor.PID_velocity.P = 10;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0.001;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  motor.PID_velocity.limit = 500.0;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01; // 0.01
  
  // // angle loop PID
  // motor.P_angle.P = 10.0;
  // motor.P_angle.I = 1.0;
  // motor.P_angle.D = 0.0;
  // motor.P_angle.output_ramp = 0.0;
  // motor.P_angle.limit = 10.0;
  // // Low pass filtering time constant
  // motor.LPF_angle.Tf = 0.3;
  
  // // current q loop PID
  // motor.PID_current_q.P = 3.0;
  // motor.PID_current_q.I = 300.0;
  // motor.PID_current_q.D = 0.0;
  // motor.PID_current_q.output_ramp = 0.0;
  // motor.PID_current_q.limit = 12.0;
  
  // // Low pass filtering time constant
  // motor.LPF_current_q.Tf = 0.005;
  
  // // current d loop PID
  // motor.PID_current_d.P = 3.0;
  // motor.PID_current_d.I = 300.0;
  // motor.PID_current_d.D = 0.0;
  // motor.PID_current_d.output_ramp = 0.0;
  // motor.PID_current_d.limit = 12.0;
  
  // // Low pass filtering time constant
  // motor.LPF_current_d.Tf = 0.005;
  
  
  // Limits
  motor.velocity_limit = 1000.0;
  motor.voltage_limit = 6;

  motor.current_limit = 5.0;

  //  general settings
  //  pwm modulation settings
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // FOCModulationType::SinePWM;
  motor.modulation_centered = 1.0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hello from Nucleo F446RE");

  SimpleFOCDebug::enable(&Serial);

  Serial.println("Starting Motor configuration...");

  encoderA.init(&spiConnectionEncoderA);
  encoderB.init(&spiConnectionEncoderB);

  // DRV8305 Inicialization
  spiConnectionDRV.begin();
  spiConnectionDRV.setBitOrder(MSBFIRST);
  spiConnectionDRV.setDataMode(SPI_MODE1);
  spiConnectionDRV.setClockDivider(SPI_CLOCK_DIV8);

  drv8305.Init();

  motor.linkSensor(&encoderA);

  driver.voltage_power_supply = 14.4;
  driver.voltage_limit = 7;

  driver.pwm_frequency = 20000;
  driver.dead_zone = 0.05;

  driver.init();

  motor.linkDriver(&driver);
  configureMotor();
 
  current_sense.linkDriver(&driver);

  motor.init();

  current_sense.init();
  motor.linkCurrentSense(&current_sense);
  
  // skip alignment
  current_sense.skip_align = true;

  // initialize motor
  
  motor.initFOC();
  
  //commander.add('M', doMotor, "Motor");

  _delay(1000);

  Can1.begin();
  Can1.setBaudRate(1000000);
  //Can1.setAutoBusOffRecovery(true);

  Serial.println("Motor ready.");
}

bool HUB_READY = false;

float link_angle = 0;

float shift_angle(float raw_angle, float shift_value) {

  float sh_angle = raw_angle + shift_value;

  if(sh_angle > 1.5*PI) {
    sh_angle = sh_angle - 2*PI;
  } else if(sh_angle > PI) {
    sh_angle = PI - sh_angle;
  } else if(sh_angle < -PI) {
    sh_angle = 2*PI + sh_angle;
  }

  if(REVERSE_ANGLES) {
    sh_angle = -sh_angle;
  }
  return sh_angle;
}

float angle_to_degrees(float angle) {
  return angle * (180.0 / PI);
}

// Функция для преобразования float в массив byte
void floatToBytes(float val, byte* bytes_array) {
  // Используем union для преобразования float в массив byte без копирования памяти
  union {
    float float_variable;
    byte byte_array[4];
  } data;

  data.float_variable = val;

  // Копируем байты из union в предоставленный массив
  for (int i = 0; i < 4; i++) {
    bytes_array[i] = data.byte_array[i];
  }
}

void can_send_angle(float angle) {
  CAN_TX_msg.id = DRIVE_ANGLE_MSG;

  CAN_TX_msg.len = 4;

  floatToBytes(angle, CAN_TX_msg.buf);

  if(Can1.write(CAN_TX_msg)) {
    //Serial.println("can send ok");
  } else {
    //Serial.println("can send fail");
  }
}

void can_send_ready() {
  CAN_TX_msg.id = DRIVE_READY_MSG;
  CAN_TX_msg.len = 1;
  CAN_TX_msg.buf[0] =  0x00;

  // if(Can1.write(CAN_TX_msg)) {
  //   Serial.println("can send ok");
  // } else {
  //   Serial.println("can send fail");
  // }
}

float last_angle = 0;
long update_timer = micros();

GMedian<10, float> filter;

float shifted_link_angle = 0;
float angle_degrees = 0;
float move_velocity = 0;

void calc_move_params() {
  link_angle = encoderB.getSensorAngle();

  Serial.print(link_angle);
  Serial.print(", ");

  shifted_link_angle = shift_angle(link_angle, ANGLE_SHIFT);
  angle_degrees = angle_to_degrees(shifted_link_angle);

  Serial.print(angle_to_degrees(link_angle)); // Вывод угла в градусах
  Serial.print(", ");
  Serial.print(angle_to_degrees(shifted_link_angle)); // Вывод угла в градусах

  float period_sec = 5000 / 10e6;

  float dx = shifted_link_angle - last_angle;

  last_angle = shifted_link_angle;

  move_velocity = dx / period_sec;
  move_velocity = filter.filtered(move_velocity);
}

void move_tick() {
  if(micros() - update_timer >= 5000) {
    update_timer = micros();

    calc_move_params();
    can_send_angle(angle_degrees);

    moveController.moveVersion1(angle_degrees, move_velocity);

    Serial.println();
  }
}

void can_read() {
  if (Can1.read(CAN_RX_msg) ) {
    Serial.print("Channel:");
    Serial.print(CAN_RX_msg.bus);

    if (CAN_RX_msg.flags.extended == false) {
      Serial.print(" Standard ID:");
    }
    else {
      Serial.print(" Extended ID:");
    }

    Serial.print(CAN_RX_msg.id, HEX);

    Serial.print(" DLC: ");
    Serial.print(CAN_RX_msg.len);

    if (CAN_RX_msg.flags.remote == false) {

      if(CAN_RX_msg.id == HUB_READY_MSG) {
        HUB_READY = true;

        can_send_ready();

        Serial.println("Hub is ready");
      }

      Serial.print(" buf: ");
      for(int i=0; i<CAN_RX_msg.len; i++) {
        Serial.print("0x"); 
        Serial.print(CAN_RX_msg.buf[i], HEX); 
        if (i != (CAN_RX_msg.len-1))  Serial.print(" ");
      }
      Serial.println();
    } else {
       Serial.println(" Data: REMOTE REQUEST FRAME");
    }
  }
}

long can_read_timer = micros();

void loop() {
  motor.loopFOC();

  move_tick();
}