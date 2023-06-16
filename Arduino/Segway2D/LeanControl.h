#ifndef _LEAN_CONTROL_H_
#define _LEAN_CONTROL_H_

#ifdef ARDUINO && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class LeanControl
{
	private:
		double Kp;
		double Kd;
		double Ki;
		static double phi_err_integ;
    static double phi_err_der;
		static double t_prev;
    static double phi_prev;
	public:
		double AngleControl(double phi_ref, double phi);

		LeanControl(double Kp = 0, double Ki = 0, double Kd = 0);	
};

#endif