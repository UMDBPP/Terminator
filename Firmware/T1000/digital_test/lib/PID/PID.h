#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

typedef struct {

	/* Controller gains */
	int32_t Kp;
	int32_t Ki;
	int32_t Kd;

	/* Derivative low-pass filter time constant */
	int32_t  tau;

	/* Output limits */
	int32_t limMin;
	int32_t limMax;
	
	/* Integrator limits */
	int32_t limMinInt;
	int32_t limMaxInt;

	/* Sample time (in seconds) */
	int32_t T;

	/* Controller "memory" */
	int32_t integrator;
	int32_t prevError;			/* Required for integrator */
	int32_t differentiator;
	int32_t prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	int32_t  out;

} PIDController;

void  PIDController_Init(PIDController *pid);
int32_t PIDController_Update(PIDController *pid, int32_t setpoint, int32_t measurement);

#endif
