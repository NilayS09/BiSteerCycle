function zdot = BiSteer3D_Dynamics(t,z,Ip,Iw,M,m,g,l,r,lF,lR)

    phi = z(1); phidot = z(2);
    delF = z(3); delR = z(4);
    x = z(5); y = z(6);
    psi = z(7); Vr = z(8);

    zdot = zeros(8,1);
    zdot(1) = phidot;

    phi_des = 0.2*cos(t); phidot_des = 0;

    Kpr = -20; Kdr = -3;
    Kpf = -1.5; Kdf = -0.5;

    delF_dot = Kpf*(phi_des - phi) + Kdf*(phidot_des - phidot);
    delR_dot = 0;
    
    % if(delF == pi/2)    
    %     Vf_dot = Kpf*(phi_des - phi) + Kdf*(phidot_des - phidot);
    %     if(delR == pi/2)
    %         Vr_dot = Kpr*(phi_des - phi) + Kdr*(phidot_des - phidot);
    %     else
    %         Vr = 0; Vr_dot = 0;
    %     end
    % else
    %     Vr_dot = Kpr*(phi_des - phi) + Kdr*(phidot_des - phidot);
    %     Vf_dot = FrontWheelConstraint(Vr,Vr_dot,delF,delR,delF_dot,delR_dot);
    % end
    Tf = 0;%Kpf*(phi_des - phi) + Kdf*(phidot_des - phidot);
    Tr = 0;%Kpr*(phi_des - phi) + Kdr*(phidot_des - phidot);
    [x_dot,y_dot,psi_dot,Vr_dot] = BiSteer2D_PoseDerivatives(Ip,Tf,Tr,...
                                   Vr,delF,delR,delF_dot,delR_dot,lF,lR,m,psi);
    Ac = BiSteer2D_COM_Acceleration(Vr,Vr_dot,delF,delR,delF_dot,delR_dot,lF,lR,psi);
    %Ac = -gain*[phi;phidot];
    phiddot = TiltAngularAcceleration(Ac,Ip,Iw,M,g,l,m,phi,r);
    zdot(2:4) = [phiddot ;delF_dot; delR_dot];
    zdot(5:8) = [x_dot; y_dot; psi_dot; Vr_dot];
    
end