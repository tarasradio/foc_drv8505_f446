#include <Arduino.h>
#include <SimpleFOC.h>
#include <definitions.h>

#define MOVE_STATE_IDLE 0
#define MOVE_STATE_UP 1
#define MOVE_STATE_DOWN 2

class MoveController {
private:
  BLDCMotor *motor;
  long move_timer;
  int move_state;

  int state = 0;

  float target_speed = 0;
  float target_angle = 0;

  // Переменные для move v2
  const float amplitude = 30.0; // Амплитуда колебаний (половина размаха), в градусах
  const float offset = -30.0; // Смещение (среднее значение угла), в градусах
  const unsigned long periodMs = 2000; // Период колебаний (время одного цикла), в миллисекундах
  const unsigned long timeStepMs = 5; // Шаг по времени в миллисекундах

  PIDController *angle_pid;
public:
  MoveController(BLDCMotor *motor);
  void init();
  void moveVersion1(float angle_degrees, float move_velocity);
  void moveVersion2(float angle_degrees, float move_velocity);
  void moveVersion3(float angle_degrees, float move_velocity);
  void moveWithPID(float angle_degrees);
  void moveUp();
  void moveDown();
};