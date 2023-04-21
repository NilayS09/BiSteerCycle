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
Vr0 = 3; delF0 = pi/3; delR0 = -pi/3;

z0 = [x0;y0;psi0;Vr0;delF0;delR0];

%% Dynamics

tend = 500;
tspan = [0 tend];
f = @(t,z) BiSteer2D_Dynamics(t,z,I33,lF,lR,m);
small = 1e-9;
options = odeset('AbsTol', small, 'RelTol', small);
solution = ode45(f, tspan, z0, options);
toutput = linspace(solution.x(1),solution.x(end),5000);
Z = deval(solution,toutput);
xR = Z(1,:); yR = Z(2,:); psi = Z(3,:);
Vr = Z(4,:); delF = Z(5,:); delR = Z(6,:);

%% For Debugging
delF_dot = -(pi/4)*cos(toutput);
delR_dot = -(pi/3)*cos(toutput);
Tf = 0; Tr = 0;
[num,den,Vr_dot] = Debug_Vr(I33,Tf,Tr,Vr,delF,delR,delF_dot,delR_dot,lF,lR,m);

% idx = den<=0;
% den(idx) = [];
% num(idx) = [];
% Vr_dot(idx) = [];
% toutput(idx) = [];
f1 = figure;
subplot(3,1,1)
plot(toutput,num)
title("numerator of Vr dot")
subplot(3,1,2)
plot(toutput,den)
title("denominator of Vr dot")
subplot(3,1,3)
plot(toutput,Vr_dot)
title("Vr dot")

%% Plots
f2 = figure;
subplot(3,2,1)
plot(toutput,xR) %delF
title("xR")
subplot(3,2,2)
plot(toutput,yR) %delR
title("yR")
subplot(3,2,3)
plot(toutput,psi) %delF
title("psi")
subplot(3,2,4)
plot(toutput,Vr) %delR
title("Vr")
subplot(3,2,5)
plot(toutput,delF) %delF
title("delF")
subplot(3,2,6)
plot(toutput,delR) %delR
title("delR")
f3 = figure;
xF = xR + (lR+lF)*cos(psi);
yF = yR + (lR+lF)*sin(psi);
plot(xF,yF) %delR
title("xF vs yF")
axis equal
f4 = figure;
plot(xR,yR) %delR
title("xR vs yR")
axis equal

f5 = figure;
xG = xR + lR*cos(psi);
yG = yR + lR*sin(psi);
% id = find(idx == 1);
% idx(id(1)) = 1;
% xG(idx) = [];
% yG(idx) = [];
plot(xG,yG) %delR
title("xG vs yG")
axis equal

%% Animation

% Defining system graphics
% system variable
L = lR + lF; r = 0.25;
RwL = L/4 ; 
FwL = L/4 ; 

% system element
axle0 = shape(L,r);
axle0 = translate(axle0,[0;-L/2]);
axle0 = rotate(axle0,-pi/2);
Rwheel0 = shape(RwL,r/2);
Rwheel0 = translate(Rwheel0,[0;-RwL/2]);
Rwheel0 = rotate(Rwheel0,-pi/2);
Fwheel0 = shape(FwL,r/2);
Fwheel0 = translate(Fwheel0,[0;-FwL/2]);
Fwheel0 = rotate(Fwheel0,-pi/2);

% Initial positions of wheels and axle
offset_angle = pi/2;
axle_initial = rotate(axle0,psi0);
xG0 = xG(1); yG0 = yG(1);
axle_initial = translate(axle_initial,[xG0;yG0]);

Rwheel_initial = rotate(Rwheel0,psi0+delR0);
xR0 = xR(1); yR0 = yR(1);
Rwheel_initial = translate(Rwheel_initial,[xR0;yR0]);

Fwheel_initial = rotate(Fwheel0,psi0+delF0);
xF0 = xF(1); yF0 = yF(1);
Fwheel_initial = translate(Fwheel_initial,[xF0;yF0]);

f6 = figure;
A = patch(axle_initial(1,:),axle_initial(2,:),'r');
Rw = patch(Rwheel_initial(1,:),Rwheel_initial(2,:),'k');
Fw= patch(Fwheel_initial(1,:),Fwheel_initial(2,:),'k');
hold on
G = scatter(xG0,yG0,30,'green','filled');
G_trace = animatedline(xG0,yG0,'Color','green');

xmin = min(xG)-(lR+lF);
xmax = max(xG)+(lR+lF);
x_axis_lim = [xmin xmax];

ymin = min(yG)-(lR+lF);
ymax = max(yG)+(lR+lF);
y_axis_lim = [ymin ymax];

xr2 = xmin:1:xmax;
yr2 = tan(pi/2 + psi0 + delR0)*(xr2 - xR0) + yR0;
r2_plot = plot(xr2,yr2,'Color','k',LineWidth=0.1);
xf2 = xr2;
yf2 = tan(pi/2 + psi0 + delF0)*(xf2 - xF0) + yF0;
f2_plot = plot(xf2,yf2,'Color','k',LineWidth=0.1);

rR_C = RotationCentre(delF0,delR0,lF,lR,psi0);
xC = xR0 - rR_C(1); yC = yR0 - rR_C(2);
C = scatter(xC,yC,50,"blue","filled");

axis equal

axis([x_axis_lim y_axis_lim])

% animation parameters
animspeed = 1;
tic 
t = toc*animspeed;

while(t<tend)
    % computing states at time 't'
    Z = deval(solution,t);

    % unpacking parameters
    xR = Z(1); yR = Z(2); psi = Z(3);
    Vr = Z(4); delF = Z(5); delR = Z(6);
    xG = xR + lR*cos(psi);
    yG = yR + lR*sin(psi);
    xF = xR + (lR+lF)*cos(psi);
    yF = yR + (lR+lF)*sin(psi);

    % Calculating new position of system elements
    axle = rotate(axle0,psi);
    axle = translate(axle,[xG;yG]);

    Rwheel = rotate(Rwheel0,psi+delR);
    Rwheel = translate(Rwheel,[xR;yR]);

    Fwheel = rotate(Fwheel0,psi+delF);
    Fwheel = translate(Fwheel,[xF;yF]);

    yr2 = tan(pi/2 + psi + delR)*(xr2 - xR) + yR;
    yf2 = tan(pi/2 + psi + delF)*(xf2 - xF) + yF;
    
    rR_C = RotationCentre(delF,delR,lF,lR,psi);
    xC = xR - rR_C(1); yC = yR - rR_C(2);

    % Reflecting new positions in graph
    A.XData = axle(1,:); A.YData = axle(2,:);
    Rw.XData = Rwheel(1,:); Rw.YData = Rwheel(2,:);
    Fw.XData = Fwheel(1,:); Fw.YData = Fwheel(2,:);
    r2_plot.YData = yr2; f2_plot.YData = yf2; 
    C.XData = xC; C.YData = yC; 
    G.XData = xG; G.YData = yG;
    addpoints(G_trace,xG,yG);

    drawnow
    t = toc*animspeed;

end

