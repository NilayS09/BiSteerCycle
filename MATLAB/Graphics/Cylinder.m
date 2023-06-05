function rod = Cylinder(r,h,C)

    [X,Y,Z] = cylinder(r);
    Z = Z*h;

    rod = surf(Y,Z,X,'FaceColor',C);
    
end