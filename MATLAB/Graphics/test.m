clc; clear; close all;

figure

% box = Cuboid(1,1,1);
rod = Cylinder(1,8,"k");
% R = [1 0 0;
%      0 0 -1;
%      0 1 0];
% RotateCylinder(R,rod)
i0 = [rod.XData;rod.YData;rod.ZData]; 
TranslateCylinder([0,-4,0],rod,i0);
view(3)
axis([-5 5 -5 5 -5 5])