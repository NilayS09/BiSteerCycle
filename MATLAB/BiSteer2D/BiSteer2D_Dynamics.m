function z_dot= BiSteer2D_Dynamics(t,z,I33,lF,lR,m)

    z_dot = zeros(6,1);
    Tf = 0; Tr = 0;
    delF_dot = -(pi/3)*cos(t); 
    delR_dot = -(pi/4)*cos(t);
    psi = z(3); Vr = z(4);
    delF = z(5); delR = z(6);
    [z_dot(1),z_dot(2),z_dot(3),z_dot(4)] = StateDer(I33,Tf,Tr,Vr,delF,...
                                            delR,delF_dot,delR_dot,lF,...
                                            lR,m,psi);

    z_dot(5) = delF_dot;
    z_dot(6) = delR_dot;

end