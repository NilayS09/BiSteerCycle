#include "DCMotorSpeedControl.h"

double DCMotorSpeedControl::w_err_integ {0};
double DCMotorSpeedControl::t_prev {0};

DCMotorSpeedControl::DCMotorSpeedControl(double Kp_val, double Ki_val, double Kd_val)
	:Kp{Kp_val}, Ki{Ki_val}, Kd{Kd_val}{
	}

double DCMotorSpeedControl::SpeedControl(double w_ref, double w){
	unsigned long t = micros();
	float dt = (t - t_prev)*1e-6f;
	if (dt < 0.0f || dt > 0.5f) dt = 1e-3f;
	double w_err = w_ref - w;
	w_err_integ = w_err_integ + w_err*dt; 
	t_prev = t;
	return Kp*(w_err) + Ki*(w_err_integ) + Kd*(-w/dt);
}
