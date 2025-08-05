#include "PID.h"

#include <stdint.h>

void PIDController_Init(PIDController *pid) {

  /* Clear controller variables */
  pid->integrator = 0;
  pid->prevError = 0;

  pid->differentiator = 0;
  pid->prevMeasurement = 0;

  pid->out = 0;
}

static int32_t error = 0;

int32_t PIDController_Update(PIDController *pid, int32_t setpoint,
                             int32_t measurement) {

  error = setpoint - measurement;

  pid->integrator = pid->integrator + ((error + pid->prevError) / (int32_t)100);

  // Anti-wind-up via integrator clamping
  if (pid->integrator > pid->limMaxInt) {

    pid->integrator = pid->limMaxInt;

  } else if (pid->integrator < pid->limMinInt) {

    pid->integrator = pid->limMinInt;
  }

  // Compute output and apply limits
  pid->out = (error / 30) + (pid->integrator);

  if (pid->out > pid->limMax) {

    pid->out = pid->limMax;

  } else if (pid->out < pid->limMin) {

    pid->out = pid->limMin;
  }

  /* Store error and measurement for later use */
  pid->prevError = error;
  pid->prevMeasurement = measurement;

  /* Return controller output */
  return pid->out;
}
