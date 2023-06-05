%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Nilay Srivastava
% Date Created : 21 April 2023
% Date Modified : 05 June 2023

% Description : Generates EOM for a 2D Segway based on acceleration of the
%               base of the pendulum

% Usage : Segway2D_EOM.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear; close all;

i = [1;0;0]; j = [0;1;0]; k = [0;0;1];

syms phi real % tilt angle
syms phidot real % tilt angular velocity
syms phiddot real % tilt angular acceleration

% unit vector along the pendulum
er =  cos(phi)*i + sin(phi)*j;

% tangential vector to pendulum
et = cross(k,er);
syms l positive % length of pendulum upto COM
Agc = -l*(phidot^2)*er + l*phiddot*et;

syms Ac real% acceleration of base
Ag = Ac*j + Agc;

% Moments
Rgc = l*er; 
syms M positive % mass of pendulum
syms g positive % acceleration due to gravity
Mc = cross(Rgc,-M*g*i);

% Rate of change of angular momentum
syms Ip positive % moment of inertia of pendulum about COM
syms Iw positive % moment of inertia of wheel
syms m positive % mass of wheel
syms r positive % radius of wheel

Rg = Rgc + r*i;
HdotC = Ip*phiddot*k + cross(Rgc,Ag)*M...
        +Iw*(Ac/r)*k + cross(Rg,Ac*j)*m;

% Equations of Motion
eqn = Mc - HdotC; eqn = simplify(eqn(3)); 
phiddot = solve(eqn,phiddot);

matlabFunction(phiddot,'file','D:\NilayFilesDocs\IISc\MTech_Final_Project\BiSteerCycle\MATLAB\Segway2D\TiltAngularAcceleration.m');