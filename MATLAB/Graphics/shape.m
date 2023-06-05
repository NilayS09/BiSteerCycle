function pendulum = shape(L,r)
% Given a pendulum length(L) and wheel radius(r) generates a vector 
% representing points describing shape of a pendulum bar

theta = 0:0.01:pi;
pendulum_width = r/3;  
upper_arc = [ cos(theta)
              sin(theta)]*pendulum_width;
lower_arc = [ cos(theta)
              sin(theta)]*pendulum_width*(-1);
left_side_line = [ zeros(1,length(theta)) - pendulum_width
                         linspace(L,0,length(theta))     ];
right_side_line = [ zeros(1,length(theta)) + pendulum_width
                         linspace(0,L,length(theta))      ];
pendulum = [upper_arc+[0;L] left_side_line lower_arc right_side_line];
end