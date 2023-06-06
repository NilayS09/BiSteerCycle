function [delFdot, delRdot, Tr] = BiSteer2D_TrajectoryController(pose_des,pose,pose_der,gains,delF,delR)

    xdes = pose_des(1); ydes = pose_des(2); psides = pose_des(3);
    x = pose(1); y = pose(2); psi = pose(3);
    xdot = pose_der(1); ydot = pose_der(2); psidot = pose_der(3); 

    [xdes_body,ydes_body] = WorldToBody2D(xdes,ydes,psi);
    [x_body,y_body] = WorldToBody2D(x,y,psi);
    [xdot_body,ydot_body] = WorldToBody2D(xdot,ydot,psi);

    x_err = xdes_body - x_body; 
    y_err = ydes_body - y_body; 
    psi_err = psides - psi;

    Kp_x = gains.x.Kp; Kp_y = gains.y.Kp; Kp_psi = gains.psi.Kp;
    Kd_x = gains.x.Kd; Kd_y = gains.y.Kd; Kd_psi = gains.psi.Kd;

    PIDx = Kp_x*x_err + Kd_x*(-xdot_body);
    PIDy = Kp_y*y_err + Kd_y*(-ydot_body);
    PIDpsi = Kp_psi*psi_err + + Kd_psi*(-psidot);

    delFdot = PIDy*cos(delF) + PIDpsi - PIDx*sin(delF) ;
    delRdot = PIDy*cos(delR) - PIDpsi - PIDx*sin(delR) ;
    Tr = PIDx + PIDy;

end