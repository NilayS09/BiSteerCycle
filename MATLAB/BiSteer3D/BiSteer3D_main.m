%%% Author: Nilay Srivastava
%%% Date Created: 28/04/2023
%%% Date Modified: 05/06/2023

%%% Description
% This script runs 3D BiSteer balance animation based on the front wheel and
% rear wheel velocity and steering angles

% Conventions
%
% delF - front wheel steering angle
% Vf - front wheel velocity
% delR - rear wheel steering angle
% Vr - rear wheel velocity
% Ac - acceleration of point C that lies directly below the COM of the
%      vehicle and on the line joining the centre of front and rear wheel
% lF - distance between front wheel and C
% lR - distance between rear wheel and C
% l - length of pendulum upto COM
% phi - Tilt angle of the segway
%
% _dot appended to any variable refers to the derivative of that variable
% _ddot appended to any variable refers to the double derivative of that
% variable

%%% Usage
% BiSteerCycle.m

%% Clearing Consoles
clc; clear; close all

%% Variable Initializations

% Front Wheel
delF = 0; delF_dot = 0;
Vf = 0; Vf_dot = 0;
% Rear Wheel
delR = 0; delR_dot = 0; 
Vr = 3; Vr_dot = 0;
% Heading Angle
psi = 0;
% Pose
x = 0; y = 0;
% Tilt Angle
phi = 0.2; 
phidot = 0;
% System 
M = 2; Ip = 2e-2; lF = 0.1; lR = 0.1; l = 0.1;
m = 0.5; Iw = 0.1e-3; r = 0.0275;
% Environment
g = 10;

%% Controller

% syms phi_s phidot_s Ac
% phiddot_s = TiltAngularAcceleration(Ac,Ip,M,g,l,phi_s);
% state_dot = [phidot_s, phiddot_s];
% state = [phi_s, phidot_s];
% A = eval(subs(jacobian(state_dot,state),state,[0 0]));
% B = eval(subs(jacobian(state_dot,Ac),phi_s,0));
% Q = diag([1 0.1]);
% R = 1;
% gain = lqr(A,B,Q,R)

%% Solver

z0 = [phi phidot delF delR x y psi Vr];
tspan = [0 100];
f = @(t,z) BiSteer3D_Dynamics(t,z,Ip,Iw,M,m,g,l,r,lF,lR);
small = 1e-5;
options = odeset('AbsTol', small, 'RelTol', small);
solution = ode45(f, tspan, z0, options);
toutput = linspace(solution.x(1),solution.x(end),500);
Z = deval(solution,toutput);

%% Plots
f1  = figure;
phi = Z(1,:); phidot = Z(2,:);
delF = Z(3,:); delR = Z(4,:);
x = Z(5,:); y = Z(6,:);
psi = Z(7,:); Vr = Z(8,:);
Vf = Vr.*(cos(delR)./cos(delF));
subplot(3,3,1)
plot(toutput,phi);
xlabel("t (seconds)")
ylabel("\phi (radians)")
subplot(3,3,2)
plot(toutput,phidot)
xlabel("t (seconds)")
ylabel("$\dot{\phi} (radians/s)$",'Interpreter','latex')
subplot(3,3,3)
plot(toutput,Vf);
xlabel("t (seconds)")
ylabel("$V_F$ (m/s)",'Interpreter','latex')
subplot(3,3,4)
plot(toutput,delF)
xlabel("t (seconds)")
ylabel("$\delta_F$ (radians)",'Interpreter','latex')
subplot(3,3,5)
plot(toutput,Vr);
xlabel("t (seconds)")
ylabel("$V_R$ (m/s)",'Interpreter','latex')
subplot(3,3,6)
plot(toutput,delR)
xlabel("t (seconds)")
ylabel("$\delta_R$ (radians)",'Interpreter','latex')
subplot(3,3,7)
plot(toutput,x)
xlabel("t (seconds)")
ylabel("$x (m)$",'Interpreter','latex')
subplot(3,3,8)
plot(toutput,y)
xlabel("t (seconds)")
ylabel("$y (m)$",'Interpreter','latex')
subplot(3,3,9)
plot(toutput,psi)
xlabel("t (seconds)")
ylabel("$\psi$ (radians)",'Interpreter','latex')

%% Animation
f2 = figure;
tend = tspan(2);
animspeed = 1;
save = 1;
BiSteer3D_Animation(solution,lR,lF,l,tend,animspeed,save)
