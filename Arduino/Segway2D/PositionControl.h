#ifndef _POSITION_CONTROL_H_
#define _POSITION_CONTROL_H_

#ifdef ARDUINO && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class PositionControl
{
	private:
		double Kp;
		double Kd;
		double Ki;
		static double x_err_integ;
    static double x_err_der;
		static double t_prev;
    static double x_prev;
	public:
		double position_Control(double x_ref, double x);

		PositionControl(double Kp = 0, double Ki = 0, double Kd = 0);	
};

#endif