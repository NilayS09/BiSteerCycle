%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Nilay Srivastava
% Date Created : 20 March 2023
% Date Modified : 21 April 2023

% Description : Generates EOM for a 2D Bisteer cycle 
% Usage : BiSteer2D_EOM.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear; close all;

% Inertial Coordinate
i = [1;0;0]; j = [0;1;0]; k = [0;0;1];

% Body Coordinates
syms psi real% heading angle
a1 = cos(psi)*i + sin(psi)*j;
a2 = -sin(psi)*i + cos(psi)*j;

% Front Wheel Coordinate
syms delF real% Front wheel angle
f1 = cos(delF)*a1 + sin(delF)*a2;
f2 = -sin(delF)*a1 + cos(delF)*a2;

% Rear Wheel Coordinate
syms delR real % Rear wheel angle
r1 = cos(delR)*a1 + sin(delR)*a2;
r2 = -sin(delR)*a1 + cos(delR)*a2;

syms Vr       real % Velocity of rear wheel
syms Vr_dot   real % Acceleration of rear wheel
syms psi_dot  real % Body Yaw Rate
syms delR_dot real % Rear wheel angular velocity
syms delF_dot real % Front wheel angular velocity
syms lR       real % distance between COM(G) and
                   % rear wheel R
syms lF       real % distance between COM(G) and
                   % rear wheel F

% Heading Angular Velocity(psi_dot) and acceleration(psiddot)
vec_Vr = Vr*r1;
vec_Vf = vec_Vr + (lR+lF)*psi_dot*a2;
expr = dot(vec_Vf,f2);
psi_dot = simplify(solve(expr,psi_dot));

vars = [Vr,delF,delR];
vars_dot = [Vr_dot,delF_dot,delR_dot]';

J = jacobian(psi_dot,vars);
psi_ddot = simplify(J*vars_dot);

% Acceleration of COM
% vG_O = Vr*r1 + lR*psi_dot1*a2;
% 
% vars = [Vr,delR,psi,psi_dot1];
% vars_dot = [Vr_dot,delR_dot,psi_dot1,psi_ddot1]';
% 
% J = jacobian(vG_O,vars);
% 
% aG_O = simplify(J*vars_dot);

aG_O = Vr_dot*r1 + Vr*(psi_dot+delR_dot)*r2 + ...
       lR*psi_ddot*a2 - lR*(psi_dot^2)*a1;

% Centre of Rotation(C)
syms q1 real % distance between R and C
syms q2 real % distance between F and C

eqn = -q1*r2 + (lR+lF)*a1 + q2*f2;
vars = [q1,q2];
[q1,q2] = solve(eqn,vars);

rR_C = -q1*r2; % distance vector b/w R and C
rF_C = -q2*f2; % distance vector b/w F and C

% Sum of Moments
syms Tf real % Forward drive force
syms Tr real % Rear drive force

MC = cross(rF_C,Tf*f1) + cross(rR_C,Tr*r1);

% Rate of change of angular momentum
syms m real % mass of BiSteer cycle 
syms I33 real % moment of inertia

rG_C = rR_C + lR*a1;
HC_dot = cross(rG_C,m*aG_O) + I33*psi_ddot*k;

% Angular Momentum Balance
eqn = MC - HC_dot; 

Vr_dot = solve(eqn(3),Vr_dot);
Vr_dot = simplify(Vr_dot);


x_dot = Vr*cos(psi + delR);
y_dot = Vr*sin(psi + delR);

matlabFunction(x_dot,y_dot,psi_dot,Vr_dot,'File','BiSteer2D_PoseDerivatives.m');
matlabFunction(rR_C,'File','BiSteer2D_RotationCentre.m');
matlabFunction(rR_C,'File','BiSteer2D_COM_Acceleration.m');