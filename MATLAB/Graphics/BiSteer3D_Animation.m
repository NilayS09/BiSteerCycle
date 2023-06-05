function BiSteer3D_Animation(solution,lR,lF,lp,tend,animspeed)
    
    % Wheel parameter
    r = 0.06; % wheel radius
    ww = 0.01; % wheel width

    % spokes parameter
    w = 2*r; % spoke width 
    d = ww ; % spoke depth
    h = d/2; % spoke height

    % Defining graphic objects
    % Wheel1
    Wheel1 = Cylinder(r,ww,'k'); % Creating cylinder object
    i0_w1 = [Wheel1.XData;Wheel1.YData;Wheel1.ZData];  
    Tvec = [0;-h;0];
    TranslateCylinder(Tvec,Wheel1,i0_w1);
    i0_w1 = [Wheel1.XData;Wheel1.YData;Wheel1.ZData];
    Tvec = [-lR;0;0];
    TranslateCylinder(Tvec,Wheel1,i0_w1);

    hold on

    % Spokes Wheel1
    Spoke_w1_1 = Cuboid(h,d,w,'b');
    Spoke_w1_1_vertices = Spoke_w1_1.Vertices;
    Spoke_w1_1.Vertices = Spoke_w1_1_vertices + Tvec';

    Spoke_w1_2 = Cuboid(w,d,h,'b');
    Spoke_w1_2_vertices = Spoke_w1_2.Vertices;
    Spoke_w1_2.Vertices = Spoke_w1_2_vertices + Tvec';

    % Wheel2
    Wheel2 = Cylinder(r,ww,'k'); % Creating cylinder object
    i0_w2 = [Wheel2.XData;Wheel2.YData;Wheel2.ZData]; 
    Tvec = [0;-h;0];
    TranslateCylinder(Tvec,Wheel2,i0_w2);
    i0_w2 = [Wheel2.XData;Wheel2.YData;Wheel2.ZData];
    Tvec = [lF;0;0];
    TranslateCylinder(Tvec,Wheel2,i0_w2);

    % Spokes Wheel2
    Spoke_w2_1 = Cuboid(h,d,w,'b');
    Spoke_w2_1_vertices = Spoke_w2_1.Vertices;
    Spoke_w2_1.Vertices = Spoke_w2_1_vertices + Tvec';

    Spoke_w2_2 = Cuboid(w,d,h,'b');
    Spoke_w2_2_vertices = Spoke_w2_2.Vertices;
    Spoke_w2_2.Vertices = Spoke_w2_2_vertices + Tvec';
    
    % Axle
    l = lR+lF;
    Axle = Cylinder(r/5,l,'r'); % Creating cylinder object
    i0_axle = [Axle.XData;Axle.YData;Axle.ZData]; 
    Tvec = [0;-l/2;0];
    TranslateCylinder(Tvec,Axle,i0_axle);
    i0_axle = [Axle.XData;Axle.YData;Axle.ZData];

    theta = 0;  phi = 0; omega = pi/2;
    R = RotationMatrixGenerator(theta,phi,omega,['Y','X','Z']);
    RotateCylinder(R,Axle,i0_axle);
    i0_axle = [Axle.XData;Axle.YData;Axle.ZData];

    % Pendulum
    Pendulum = Cylinder(r/5,2*lp,'g'); % Creating cylinder object
    i0_pendulum = [Pendulum.XData;Pendulum.YData;Pendulum.ZData]; 

    theta = pi/2;  phi = 0; omega = 0;
    R = RotationMatrixGenerator(theta,phi,omega,['Y','X','Z']);
    RotateCylinder(R,Pendulum,i0_pendulum);
    i0_pendulum = [Pendulum.XData;Pendulum.YData;Pendulum.ZData];

    axis equal
    a = gca;
    a.XLim = a.XLim + [-1 1];
    a.YLim = a.YLim + [-1 1];

    xlabel("X")
    ylabel("Y")
    zlabel("Z")

    tic
    t = toc*animspeed;
    dt = 0.1;
    while(t<tend)
        Z = deval(solution,t);

        % Unpacking parameters
        tilt = Z(1);
        Vf = Z(3); delF = Z(4);
        Vr = Z(5); delR = Z(6);
        x = Z(7); y = Z(8); psi = Z(9);

        Tvec = [x,y,0];
        % Pendulum Update
        theta = -tilt;  phi = 0; omega = psi;
        R = RotationMatrixGenerator(theta,phi,omega,['Y','X','Z']);
        RotateCylinder(R,Pendulum,i0_pendulum);
        i0_pendulum_temp = [Pendulum.XData;Pendulum.YData;Pendulum.ZData];
        TranslateCylinder(Tvec,Pendulum,i0_pendulum_temp)

        % Axle Update
        theta = 0;  phi = 0; omega = psi;
        R = RotationMatrixGenerator(theta,phi,omega,['Y','X','Z']);
        RotateCylinder(R,Axle,i0_axle);
        i0_axle_temp = [Axle.XData;Axle.YData;Axle.ZData];
        TranslateCylinder(Tvec,Axle,i0_axle_temp)

        % Wheel1 Update
        theta = 0;  phi = dt*Vr/r; omega = psi+delR;
        R = RotationMatrixGenerator(theta,phi,omega,['Y','X','Z']);
        RotateCylinder(R,Wheel1,i0_w1);
        i0_w1_temp = [Wheel1.XData;Wheel1.YData;Wheel1.ZData];
        Tvec = [x-lR*cos(psi),y-lR*sin(psi),0];
        TranslateCylinder(Tvec,Wheel1,i0_w1_temp)

        % Spoke Wheel1 Update
        Spoke_w1_1.Vertices = Spoke_w1_1_vertices*R';
        Spoke_w1_1.Vertices = Spoke_w1_1.Vertices + Tvec;
        
        Spoke_w1_2.Vertices = Spoke_w1_2_vertices*R';
        Spoke_w1_2.Vertices = Spoke_w1_2.Vertices + Tvec;

        % Wheel2 Update
        theta = 0;  phi = dt*Vf/r; omega = psi+delF;
        R = RotationMatrixGenerator(theta,phi,omega,['Y','X','Z']);
        RotateCylinder(R,Wheel2,i0_w2);
        i0_w2_temp = [Wheel2.XData;Wheel2.YData;Wheel2.ZData];
        Tvec = [x+lF*cos(psi),y+lF*sin(psi),0];
        TranslateCylinder(Tvec,Wheel2,i0_w2_temp)

        % Spoke Wheel1 Update
        Spoke_w2_1.Vertices = Spoke_w2_1_vertices*R';
        Spoke_w2_1.Vertices = Spoke_w2_1.Vertices + Tvec;
        
        Spoke_w2_2.Vertices = Spoke_w2_2_vertices*R';
        Spoke_w2_2.Vertices = Spoke_w2_2.Vertices + Tvec;

        % psi_deg = psi*180/pi;
        % view(90+psi_deg,0)
        view(0,90)
        drawnow
        t = toc*animspeed;
        % a.XLim = [x-1 x+1];
        % a.YLim = [y-1 y+1];
    end
end