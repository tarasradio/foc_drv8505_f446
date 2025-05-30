#include <Arduino.h>
#include <SimpleFOC.h>
#include <GyverFilters.h>

#include "STM32_CAN.h"

#include "definitions.h"

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

void configureAmplierClamping(SPIClass spiConnectionToDRV) {
  // Clamp sense amplifier output to 3.3V

  digitalWrite(CS_DRV, LOW);
  byte resp1 = spiConnectionToDRV.transfer(B01001100);
  byte resp2 = spiConnectionToDRV.transfer(B10100000);
  digitalWrite(CS_DRV, HIGH);

  Serial.println("Configure Amplier Clamping");
  Serial.print("Byte 0 : ");
  Serial.println(resp1, BIN);
  Serial.print("Byte 1 : ");
  Serial.println(resp2, BIN);
}

void configure6PWM_Mode(SPIClass spiConnectionToDRV) {
  // Set to three PWM inputs mode to 6PWM

  digitalWrite(CS_DRV, LOW);
  byte resp1 = spiConnectionToDRV.transfer(B00111010);
  byte resp2 = spiConnectionToDRV.transfer(B0000110);
  digitalWrite(CS_DRV, HIGH);

  Serial.println("Configure 6PWM Mode");
  Serial.print("Byte 0 : ");
  Serial.println(resp1, BIN);
  Serial.print("Byte 1 : ");
  Serial.println(resp2, BIN);
}

void configure3PWM_Mode(SPIClass spiConnectionToDRV) {
  // Set to three PWM inputs mode to 3PWM

  digitalWrite(CS_DRV, LOW);
  byte resp1 = spiConnectionToDRV.transfer(B00111010); // 0 - write, 0111 - 0x7 address, 0 1 01 000 01 10
  byte resp2 = spiConnectionToDRV.transfer(B10000110);
  digitalWrite(CS_DRV, HIGH);

  Serial.println("Configure 3PWM Mode");
  Serial.print("Byte 0 : ");
  Serial.println(resp1, BIN);
  Serial.print("Byte 1 : ");
  Serial.println(resp2, BIN);
}

void drv8305Initialization()
{
  Serial.println("DRIVER: DRV8305 INIT");

  pinMode(ENA_GATE, OUTPUT);
  pinMode(CS_DRV, OUTPUT);
  digitalWrite(CS_DRV, HIGH);

  configure6PWM_Mode(spiConnectionDRV);
  configureAmplierClamping(spiConnectionDRV);

  _delay(500);

  Serial.println("DRIVER: enGate Enabled");

  digitalWrite(ENA_GATE, HIGH);
}

BLDCMotor motor = BLDCMotor(MOTOR_PP);

BLDCDriver6PWM driver = BLDCDriver6PWM(INH_A, INL_A, INH_B, INL_B, INH_C, INL_C);

LowsideCurrentSense current_sense = LowsideCurrentSense(0.007f, 12.22f, SENSE_A, SENSE_B, SENSE_C);

void configureMotor() {
  // control loop type and torque mode
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::velocity;
  
  // controller configuration 
  // default parameters in defaults.h

  // controller configuration based on the control type 
  // velocity PID controller parameters
  // default P=0.5 I = 10 D =0
  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 5;
  motor.PID_velocity.D = 0.001;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  motor.PID_velocity.limit = 500.0;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.2; // 0.01
  
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
  motor.velocity_limit = 800.0;
  motor.voltage_limit = 5;

  motor.current_limit = 3.0;

  //  general settings
  //  pwm modulation settings
  motor.foc_modulation = FOCModulationType::SinePWM;
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

  drv8305Initialization();

  motor.linkSensor(&encoderA);

  driver.voltage_power_supply = 14.4;
  driver.voltage_limit = 5;

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

PIDController angle_pid(800.0, 5.0, 0.005, 0, 500.0);

float target_angle = 2.5;
float link_angle = 0;

float m_speed = 0;

float shift_angle(float raw_angle, float shift_value) {
  float sh_angle = raw_angle + shift_value;
  if(sh_angle > PI*2)
    sh_angle = sh_angle - PI*2;
  else if(sh_angle < 0)
    sh_angle = PI*2 - sh_angle;
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
    Serial.println("can send ok");
  } else {
    Serial.println("can send fail");
  }
}

void can_send_ready() {
  CAN_TX_msg.id = DRIVE_READY_MSG;
  CAN_TX_msg.len = 1;
  CAN_TX_msg.buf[0] =  0x00;

  if(Can1.write(CAN_TX_msg)) {
    Serial.println("can send ok");
  } else {
    Serial.println("can send fail");
  }
}

float last_angle = 0;
long update_timer = micros();

GMedian<10, float> filter;

long move_timer = millis();

#define MOVE_STATE_IDLE 0
#define MOVE_STATE_UP 1
#define MOVE_STATE_DOWN 2

int move_state = MOVE_STATE_IDLE;

void move_up() {
  Serial.println(" Move Up");
  motor.enable();
  motor.move(UP_SPEED);
}

void move_down() {
  Serial.println(" Move Down");
  motor.enable();
  motor.move(DOWN_SPEED);
}

void move_tick() {
  if(micros() - update_timer >= 5000) {
    update_timer = micros();

    link_angle = encoderB.getSensorAngle();
    float shifted_link_angle = shift_angle(link_angle, ANGLE_SHIFT);
    float angle_degrees = angle_to_degrees(shifted_link_angle);

    //if(HUB_READY == true) {
      can_send_angle(angle_degrees);
    //}

    //Serial.print(angle_to_degrees(link_angle)); // Вывод угла в градусах
    //Serial.print(", ");
    //Serial.print(angle_to_degrees(shifted_link_angle)); // Вывод угла в градусах

    float period_sec = 5000 / 10e6;

    float dx = shifted_link_angle - last_angle;

    last_angle = shifted_link_angle;

    float velocity = dx / period_sec;

    velocity = filter.filtered(velocity);

    //Serial.print(", ");
    //Serial.print(period_sec);
    //Serial.print(", ");
    //Serial.print(velocity);

    if(angle_to_degrees(shifted_link_angle) <= DOWN_LIMIT) {
      //Serial.println(" Down position limit");
      motor.disable();
      move_state = MOVE_STATE_IDLE;
    } else if (angle_to_degrees(shifted_link_angle) >= UP_LIMIT) {
      //Serial.println(" Up position limit");
      motor.disable();
      move_state = MOVE_STATE_IDLE;
    } else {
      if(velocity > 10.0) {
        if(move_state == MOVE_STATE_DOWN || move_state == MOVE_STATE_IDLE) {
          if(millis() - move_timer >= 200) {
            move_timer = millis();
            move_state = MOVE_STATE_UP;
            move_up();
          }
        }
      } else if(velocity < -10.0) {
        if(move_state == MOVE_STATE_UP || move_state == MOVE_STATE_IDLE) {
          if(millis() - move_timer >= 200) {
            move_timer = millis();
            move_state = MOVE_STATE_DOWN;
            move_down();
          }
        }
      } else {
        //Serial.println(" Idle");
        move_timer = millis();
        motor.disable();
        move_state = MOVE_STATE_IDLE;
      }
    }
    //Serial.println();
    float link_angle_error = target_angle - link_angle;
    m_speed = angle_pid(-link_angle_error);
  }
}

int state = 0;

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

  // if(micros() - can_read_timer >= 5000) {
  //   can_read_timer = micros();
  //   can_read();
  // }

  // motor.move(m_speed);

  // if(state == 0) {
  //   if(link_angle > target_angle) {
  //     target_angle = 0.6;
  //     state = 1;
  //   }
  // } else {
  //   if(link_angle < target_angle) {
  //     target_angle = 2.5;
  //     state = 0;
  //   }
  // }
}