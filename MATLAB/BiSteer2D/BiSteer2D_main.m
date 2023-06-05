%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Nilay Srivastava
% Date Created : 20 March 2023
% Date Modified : 20 March 2023

% Description : Solves EOM and plots and shows animation
%               for a 2D Bisteer cycle 
% Usage : BiSteer2D.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clearing Consoles
clc; clear; close all;

%% Initializations

% System Parameters
lR = 1; lF = 1;
m = 1; I33 = 0.1;

% States
x0 = 1; y0 = 0; psi0 = pi/2;
Vr0 = 1; delF0 = pi/4; delR0 = pi/3;

z0 = [x0;y0;psi0;Vr0;delF0;delR0];

%% Dynamics

tend = 100;
tspan = [0 tend];
f = @(t,z) BiSteer2D_Dynamics(t,z,I33,lF,lR,m);
small = 1e-9;
options = odeset('AbsTol', small, 'RelTol', small);
solution = ode45(f, tspan, z0, options);
toutput = linspace(solution.x(1),solution.x(end),5000);
Z = deval(solution,toutput);
xR = Z(1,:); yR = Z(2,:); psi = Z(3,:);
Vr = Z(4,:); delF = Z(5,:); delR = Z(6,:);

%% Plots
f1 = figure;
subplot(3,2,1)
plot(toutput,xR)
title("xR")
subplot(3,2,2)
plot(toutput,yR)
title("yR")
subplot(3,2,3)
plot(toutput,psi)
title("psi")
subplot(3,2,4)
plot(toutput,Vr)
title("Vr")
subplot(3,2,5)
plot(toutput,delF)
title("delF")
subplot(3,2,6)
plot(toutput,delR) 
title("delR")
f2 = figure;
xF = xR + (lR+lF)*cos(psi);
yF = yR + (lR+lF)*sin(psi);
plot(xF,yF) 
title("xF vs yF")
axis equal
f3 = figure;
plot(xR,yR) 
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