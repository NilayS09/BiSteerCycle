%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Nilay Srivastava
% Date Created : 20 March 2023
% Date Modified : 07 June 2023

% Description : Solves EOM and plots and shows animation
%               for a 2D Bisteer cycle 
% Usage : BiSteer2D.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clearing Consoles
clc; clear; close all;

%% Initializations

% System Parameters
lR = 1; lF = 1;
m = 1; Ip = 0.1;

% States
x0 = 1.4142; y0 = 0; psi0 = pi/2;
Vr0 = 0; delF0 = 0; delR0 = 0;
x_dot0 = 0; y_dot0 = 0; psi_dot0 = 0;

z0 = [x0;y0;psi0;Vr0;delF0;delR0;x_dot0;y_dot0;psi_dot0];

%% Dynamics

tend = 50;
tspan = [0 tend];
f = @(t,z) BiSteer2D_Dynamics(t,z,Ip,lF,lR,m);
small = 1e-9;
options = odeset('AbsTol', small, 'RelTol', small);
solution = ode45(f, tspan, z0, options);
toutput = linspace(solution.x(1),solution.x(end),5000);
Z = deval(solution,toutput);
xR = Z(1,:); yR = Z(2,:); psi = Z(3,:);
Vr = Z(4,:); delF = Z(5,:); delR = Z(6,:);
xdot = Z(7,:); ydot = Z(8,:); psidot = Z(9,:);

%% Plots
f1 = figure;
subplot(3,3,1)
plot(toutput,xR)
title("xR")
subplot(3,3,2)
plot(toutput,yR)
title("yR")
subplot(3,3,3)
plot(toutput,psi)
% hold on
% plot(toutput,pi/2+atan2(yR,xR))
% hold off
title("psi")
subplot(3,3,4)
plot(toutput,Vr)
title("Vr")
subplot(3,3,5)
plot(toutput,delF)
title("delF")
subplot(3,3,6)
plot(toutput,delR) 
title("delR")
subplot(3,3,7)
plot(toutput,xdot)
title("xdot")
subplot(3,3,8)
plot(toutput,ydot)
title("ydot")
subplot(3,3,9)
plot(toutput,psidot) 
title("psidot")
f2 = figure;
xF = xR + (lR+lF)*cos(psi);
yF = yR + (lR+lF)*sin(psi);
plot(xF,yF) 
title("xF vs yF")
axis equal
f3 = figure;
xdes = 1.4142*cos(toutput);
ydes = 1.4142*sin(toutput);
plot(ydes,xdes)
hold on
plot(xR,yR) 
hold off
title("xR vs yR")
axis equal

f4 = figure;
xG = xR + lR*cos(psi);
yG = yR + lR*sin(psi);
plot(xG,yG)
title("xG vs yG")
axis equal

%% Animation

f5 = figure("Name","BiSteer2D Animation");
animspeed = 1;
tend = tspan(2);
save = 0;
BiSteer2D_Animation(lR,lF,xR,yR,xG,yG,xF,yF,psi0,delR0,delF0,solution,animspeed,tend,save)