#include <math.h>

#include "robo_car_if/pid.h"

namespace pid {

PID::PID(Params_T* params) {
  kp_  = params->kp;
  ki_  = params->ki;
  kd_  = params->kd;
  dt_  = params->dt;
  tol_ = params->tol;

  last_e_ = 0.0f;
  integral_ = 0.0f;
}

float PID::Step(float error, float max, float min) {
  float u, d;           /* actuator command, derivative  */
  float ti = integral_; /* temporary integral            */
  float e = error;      /* error */
  int8_t sign;

  /* Ignore the error if it's smaller than the tolerance */
  if (fabs(e) < tol_) {
    e = 0.0f;
  }

  d = (e - last_e_) / dt_;
  ti += (((e + last_e_) / 2) * dt_); /* Trapezoidal integration */

  u = (kp_ * e) + (ki_ * integral_) + (kd_ * d);

  last_e_ = e;

  /* Determine the sign of u */
  sign = signbit(u) ? -1 : 1;

  /* Limit the integral term via clamping */
  if (fabs(u) < max) {
    integral_ = ti;
  }

  /* Perform saturation on the activation signal */
  if (fabs(u) < min) {
    u = sign * min;
  } else if (fabs(u) > max) {
    u = sign * max;
  }

  return (u);
}

void PID::Reset(void) {
  last_e_ = 0.0f;
  integral_ = 0.0f;
}

}  // namespace pid