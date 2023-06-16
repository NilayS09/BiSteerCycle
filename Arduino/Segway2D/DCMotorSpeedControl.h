#ifndef _DC_MOTOR_SPEED_CONTROL_H_
#define _DC_MOTOR_SPEED_CONTROL_H_

#ifdef ARDUINO && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class DCMotorSpeedControl
{
	private:
		double Kp;
		double Kd;
		double Ki;
		static double w_err_integ;
		static double t_prev;
	public:
		double SpeedControl(double w_ref, double w);

		DCMotorSpeedControl(double Kp = 0, double Ki = 0, double Kd = 0);	
};

#endif
