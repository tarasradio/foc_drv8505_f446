#include <math_helpers.h>

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