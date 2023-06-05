function  s_shape = translate(shape,shift)
% shape is a 2xn array with each row containing some x,y coordinate of the
% array shape.
% shift is 2x1 array specifying the amount by which the shape needs to move

s_shape = shape+shift;
end