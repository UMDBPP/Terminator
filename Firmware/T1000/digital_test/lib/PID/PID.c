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

  /*
   * Error signal
   */
  error = setpoint - measurement;

  /*
   * Proportional
   */
  // static int32_t proportional = pid->Kp * error;

  /*
   * Integral
   */
  pid->integrator = pid->integrator + ((error + pid->prevError) / (int32_t)100);

  /* Anti-wind-up via integrator clamping */
  if (pid->integrator > pid->limMaxInt) {

    pid->integrator = pid->limMaxInt;

  } else if (pid->integrator < pid->limMinInt) {

    pid->integrator = pid->limMinInt;
  }

  /*
   * Derivative (band-limited differentiator)
   */
  /*
    pid->differentiator =
        -(2.0f * pid->Kd *
              (measurement -
               pid->prevMeasurement) // Note: derivative on measurement,
    therefore
                                        // minus sign in front of equation!
          + (2.0f * pid->tau - pid->T) * pid->differentiator) /
        (2.0f * pid->tau + pid->T);
  */
  /*
   * Compute output and apply limits
   */
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
