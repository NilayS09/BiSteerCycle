%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Nilay Srivastava
% Date Created : 20 March 2023
% Date Modified : 20 March 2023

% Description : Solves EOM and plots and shows animation
%               for a 2D Segway 
% Usage : Segway2D_main.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clearing Consoles
clc; clear; close all;

%% Initializations
% Pendulum Parameters
M = 0.7; L = 0.163; l = 0.1; Ip = 1e-3;

% Wheel Radius
m = 0.1; r = 0.05; Iw = 0.069e-3;

% Environment Parameters
g = 10;

% Initial Conditions
phi0 = 0.1; y0 = 1; phidot0 = 0; ydot0 = 0;
z0 = [phi0 y0 phidot0 ydot0];

%% Controller

syms phi phidot y ydot Ac
yddot = Ac;
phiddot = TiltAngularAcceleration(Ac,Ip,Iw,M,g,l,m,phi,r);
state_dot = [phidot ydot phiddot yddot];
state = [phi y phidot ydot];
A = eval(subs(jacobian(state_dot,state),phi,0));
B = eval(subs(jacobian(state_dot,Ac),phi,0));
Q = diag([1 1 1 1]);
R = 1;
gain = lqr(A,B,Q,R);
%gain = 0;

%% Dynamics

tspan = [0 10];
f = @(t,z) Segway2D_Dynamics(z,Ip,Iw,M,g,l,m,r,gain);
small = 1e-9;
options = odeset('AbsTol', small, 'RelTol', small);
solution = ode45(f, tspan, z0, options);
toutput = linspace(solution.x(1),solution.x(end),500);
Z = deval(solution,toutput);

% Plot pendulum angle(theta) and wheel angle(phi)
phi = Z(1,:); y = Z(2,:);
phidot = Z(3,:); ydot = Z(4,:);
f1 = figure;
subplot(3,2,1)
plot(toutput, phi,LineWidth=2)
title("Segway Tilt Angle",LineWidth=10)
ylabel('Time(Seconds)','FontSize',10,FontWeight='bold')
ylabel('Tilt angle(radians)','FontSize',10,FontWeight='bold')
subplot(3,2,2)
plot(toutput, y,LineWidth=2)
title("Segway Position",LineWidth=10)
ylabel('Time(Seconds)','FontSize',10,FontWeight='bold')
ylabel('Position(m)','FontSize',10,FontWeight='bold')
subplot(3,2,3)
plot(toutput, phidot,LineWidth=2)
title("Segway Tilt Angular Velocity",LineWidth=10)
ylabel('Time(Seconds)','FontSize',10,FontWeight='bold')
ylabel('Tilt angular velocity(radians)','FontSize',10,FontWeight='bold')
subplot(3,2,4)
plot(toutput, ydot,LineWidth=2)
title("Wheel Linear Velocity",LineWidth=10)
ylabel('Time(Seconds)','FontSize',10,FontWeight='bold')
ylabel('Velocity(m/s)','FontSize',10,FontWeight='bold')
subplot(3,2,5)
Ac = -gain*Z;
%Ac = 19*phi+phidot;
plot(toutput, Ac,LineWidth=2)
title("Acceleration of Segway Base",LineWidth=10)
ylabel('Time(Seconds)','FontSize',10,FontWeight='bold')
ylabel('Acceleration(m/s^2)','FontSize',10,FontWeight='bold')
axis equal

%% Animation
f2 = figure("Name",'Segway2D Animation');
[segway_pendulum,wheel,spoke] = shapeSegway(L,r);
animspeed = 1;
tend = tspan(2);
save = 0;
Segway2D_Animation(segway_pendulum,wheel,spoke,solution,L,r,animspeed,tend,save)