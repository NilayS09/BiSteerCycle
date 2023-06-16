#include "MsTimer2.h"
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "LeanControl.h"
//#include "PositionControl.h"
//#include "LowPassFilter.h"
//#include "DCMotorSpeedControl.h"
//#include <Encoder.h>
//#include <math.h>

//#define CPR 8540

MPU6050 mpu;

KalmanFilter kalmanfilter;
float kalmanfilter_angle;

int16_t ax, ay, az, gx, gy, gz;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

LeanControl AccelerationL{60,0,5};
double phi_ref {0};
float phi_Ac;

//PositionControl AccelerationP{2.35,0,1.97};
// PositionControl AccelerationP{0,0,0};
// double x_ref {0};
// float x_Ac;

// float R = 0.055; //Wheel Radius

// Encoder myEnc(2, 3);

// long oldPosition = 0;
// long newPosition = 0;
// double vel;
// double tstart = 0;
// LowPassFilter LPF{0.01};
// DCMotorSpeedControl Voltage{0.1,10,0};
// double w_ref;
//float t_prev = 0;
int pwm;

void ReadStates() {
  sei();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  // double tend = millis();
  // newPosition = myEnc.read();
  // double pos_diff = 2 * PI * fmod(newPosition - oldPosition, CPR) / CPR;
  // vel = 1000 * pos_diff/ (tend - tstart);
  // if (tend - tstart == 0) vel = 0;
  // //vel = LPF.filter(vel);
  // oldPosition = newPosition;
  // tstart = tend;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  Serial.begin(9600);
  mpu.initialize();
  MsTimer2::set(5, ReadStates);
  MsTimer2::start();
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

  // High level PID  
  phi_Ac = AccelerationL.AngleControl(phi_ref, kalmanfilter_angle*PI/180.0);
  //x_Ac = AccelerationP.position_Control(x_ref, R*2*PI*newPosition/CPR);

  // Low Level PID
  float t = millis();
  //float dt = t - t_prev; 
  //w_ref += (phi_Ac + x_Ac)*dt*1e-3;
  //pwm = abs(255*(Voltage.SpeedControl(w_ref,vel)/12));
  //pwm = 0;
  //t_prev = t;

  if(phi_Ac > 0)
    digitalWrite(7,HIGH);
  else
    digitalWrite(7,LOW);

  // if(kalmanfilter_angle>1 || kalmanfilter_angle<-1)
  //   pwm = abs(phi_Ac)*23; 
  // else
  //   pwm = 0;
  pwm = abs(phi_Ac)*23;
  pwm = constrain(pwm, 0, 255);
  if(kalmanfilter_angle<60)
    analogWrite(6,pwm);
  else
    analogWrite(6,0);

  // Serial.println(kalmanfilter_angle);
  // Serial.print("          ");
  // Serial.println(pwm);
  
}
