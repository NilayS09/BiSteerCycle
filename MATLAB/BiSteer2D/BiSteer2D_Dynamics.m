function z_dot= BiSteer2D_Dynamics(t,z,Ip,lF,lR,m)

    z_dot = zeros(9,1);
    Tf = 0;
    % delF_dot = 0;%-(pi/3)*cos(t); 
    % delR_dot = 0;%-(pi/4)*cos(t);
    x = z(1); y = z(2);
    psi = z(3); Vr = z(4);
    delF = z(5); delR = z(6);
    x_dot = z(7); y_dot = z(8);
    psi_dot = z(9);

    xdes = 2*cos(0.5*t);
    ydes = 2*sin(0.5*t);

    psides = 0.5*t + pi/2;

    % if ydes >= 0
    %     psides = atan2(ydes,xdes) + pi/2;
    % else
    %     psides = atan2(ydes,xdes) + 2*pi + pi/2;
    % end
    
    pose_des = [xdes,ydes,psides];
    pose = [x,y,psi];
    pose_der = [x_dot,y_dot,psi_dot];
    gains.x = struct('Kp',16,'Kd',11,'Ki',0);
    gains.y = struct('Kp',16,'Kd',11,'Ki',0);
    gains.psi = struct('Kp',5,'Kd',2,'Ki',0);
    [delF_dot, delR_dot, Tr] = BiSteer2D_TrajectoryController(pose_des,pose,...
                           pose_der,gains,delF,delR);

    [x_dot,y_dot,psi_dot,Vr_dot] = BiSteer2D_PoseDerivatives(Ip,Tf,Tr,Vr,delF,...
                                            delR,delF_dot,delR_dot,lF,...
                                            lR,m,psi);

    [x_ddot, y_ddot, psi_ddot] = BiSteer2D_PoseDoubleDerivatives(Ip,Tf,Tr,...
                                 Vr,Vr_dot,delF,delR,delF_dot,delR_dot,lF,...
                                 lR,m,psi);

    z_dot(1) = x_dot;
    z_dot(2) = y_dot;
    z_dot(3) = psi_dot;
    z_dot(4) = Vr_dot;
    z_dot(5) = delF_dot;
    z_dot(6) = delR_dot;
    z_dot(7) = x_ddot;
    z_dot(8) = y_ddot;
    z_dot(9) = psi_ddot;

end