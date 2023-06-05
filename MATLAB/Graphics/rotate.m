function r_shape = rotate(shape,angle)
% shape is a 2xn array with each row containing some x,y coordinate of the
% array shape.
% angle is the amount by which shape needs to be rotated in its body frame

r_shape = [cos(angle) -sin(angle);
           sin(angle)  cos(angle)] * shape;
end