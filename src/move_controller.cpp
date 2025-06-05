#include <move_controller.h>

MoveController::MoveController(BLDCMotor *motor) {
  this->motor = motor;
}

void MoveController::init() {
  move_timer = millis();
  move_state = MOVE_STATE_IDLE;
  angle_pid = new PIDController(1000.0, 5.0, 0.005, 0, 1000.0);
}

void MoveController::moveVersion1(float angle_degrees, float move_velocity) {
  if(angle_degrees <= DOWN_LIMIT) {
    Serial.print(" Down position limit");
    motor->disable();
    move_state = MOVE_STATE_IDLE;
  } else if (angle_degrees >= UP_LIMIT) {
    Serial.print(" Up position limit");
    motor->disable();
    move_state = MOVE_STATE_IDLE;
  } else {
    if(move_velocity > 10.0) {
      if(move_state == MOVE_STATE_DOWN || move_state == MOVE_STATE_IDLE) {
        if(millis() - move_timer >= 200) {
          move_timer = millis();
          move_state = MOVE_STATE_UP;
          moveUp();
        }
      }
    } else if(move_velocity < -10.0) {
      if(move_state == MOVE_STATE_UP || move_state == MOVE_STATE_IDLE) {
        if(millis() - move_timer >= 200) {
          move_timer = millis();
          move_state = MOVE_STATE_DOWN;
          moveDown();
        }
      }
    } else {
      Serial.print(" Idle");
      move_timer = millis();
      motor->disable();
      move_state = MOVE_STATE_IDLE;
    }
  }
}

void MoveController::moveVersion2(float angle_degrees, float move_velocity) {
  unsigned long elapsedTimeMs = millis();  //  millis() возвращает время с момента запуска Arduino
  float elapsedTimeSeconds = (float)elapsedTimeMs / 1000.0f;
  target_angle = offset + amplitude * sin(2 * PI * elapsedTimeSeconds / (periodMs / 1000.0f));  
  // Умножаем на 2*PI для радиан, делим период на 1000, т.к. период в мс

  Serial.print(", gen angle = ");
  Serial.print(target_angle);

  if(angle_degrees < -60 || angle_degrees > 0) {
    motor->disable();
  }

  if(angle_degrees > target_angle) {
    moveDown();
  } else if(angle_degrees <= target_angle) {
    moveUp();
  }
}

void MoveController::moveVersion3(float angle_degrees, float move_velocity) {
  if(state == 0) {
    if(angle_degrees > target_angle) {
      target_angle = -0.6;
      state = 1;
    }
  } else {
    if(angle_degrees < target_angle) {
      target_angle = 0;
      state = 0;
    }
  }
}

void MoveController::moveWithPID(float angle_degrees) {
  float link_angle_error = target_angle - angle_degrees;
  target_speed = (*angle_pid)(link_angle_error);

  motor->move(target_speed);
}

void MoveController::moveUp() {
  Serial.println(" Move Up");
  motor->enable();
  motor->move(UP_SPEED);
}

void MoveController::moveDown() {
  Serial.println(" Move Down");
  motor->enable();
  motor->move(DOWN_SPEED);
}