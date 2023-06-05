function z_dot= Segway2D_Dynamics(z,Ip,Iw,M,g,l,m,r,gain)
  
    z_dot = zeros(4,1);
    phi = z(1); 
    phidot = z(3); ydot = z(4);

    z_dot(1) = phidot; z_dot(2) = ydot;

    Ac = -gain*z;
    %Ac = 19*phi + phidot;

    phiddot = TiltAngularAcceleration(Ac,Ip,Iw,M,g,l,m,phi,r);
    z_dot(3) = phiddot;

    yddot = Ac;
    z_dot(4) = yddot;

end