#include "LeanControl.h"

double LeanControl::phi_err_integ {0};
double LeanControl::phi_err_der {0};
double LeanControl::t_prev {0};
double LeanControl::phi_prev {0};

LeanControl::LeanControl(double Kp_val, double Ki_val, double Kd_val)
	:Kp{Kp_val}, Ki{Ki_val}, Kd{Kd_val}{
	}

double LeanControl::AngleControl(double phi_ref, double phi){
	unsigned long t = micros();
	float dt = (t - t_prev)*1e-6f;
	if (dt < 0.0f || dt > 0.5f) dt = 1e-3f;
	double phi_err = phi_ref - phi;
	phi_err_integ += phi_err*dt; 
  phi_err_der = (phi - phi_prev)/dt;
	t_prev = t;
  phi_prev= phi;
	return Kp*(phi_err) + Ki*(phi_err_integ) + Kd*(-phi_err_der);
}
