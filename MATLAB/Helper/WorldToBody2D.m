function [xbody,ybody] = WorldToBody2D(x,y,psi)
    
    R = [ cos(psi) sin(psi);
         -sin(psi) cos(psi)];

    body_coords = R*[x;y];
    xbody = body_coords(1);
    ybody = body_coords(2);

end