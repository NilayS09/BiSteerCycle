#include "PositionControl.h"

double PositionControl::x_err_integ {0};
double PositionControl::x_err_der {0};
double PositionControl::t_prev {0};
double PositionControl::x_prev {0};

PositionControl::PositionControl(double Kp_val, double Ki_val, double Kd_val)
	:Kp{Kp_val}, Ki{Ki_val}, Kd{Kd_val}{
	}

double PositionControl::position_Control(double x_ref, double x){
	unsigned long t = micros();
	float dt = (t - t_prev)*1e-6f;
	if (dt < 0.0f || dt > 0.5f) dt = 1e-3f;
	double x_err = x_ref - x;
	x_err_integ += x_err*dt; 
  x_err_der = (x - x_prev)/dt;
	t_prev = t;
  x_prev= x;
	return Kp*(x_err) + Ki*(x_err_integ) + Kd*(-x_err_der);
}
