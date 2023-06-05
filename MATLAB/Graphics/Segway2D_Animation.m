function Segway2D_Animation(segway_pendulum,wheel,spoke,solution,L,r,animspeed,tend,save)
% Extracts 2D coordinates of plot of Segway and wheel model
% Generating more spokes
rot_spoke_90 = rotate(spoke,pi/2);
rot_spoke_45 = rotate(spoke,pi/4);
rot_spoke_135 = rotate(spoke,3*pi/4);
% Generating plot data
Sp = plot(segway_pendulum(1,:),segway_pendulum(2,:),'Color','r','LineWidth',1);
hold on
w = plot(wheel(1,:),wheel(2,:),'Color','k','LineWidth',1);
s = plot(spoke(1,:),spoke(2,:),'Color','k','LineWidth',1);
s90 = plot(rot_spoke_90(1,:),rot_spoke_90(2,:),'Color','k','LineWidth',1);
s45 = plot(rot_spoke_45(1,:),rot_spoke_45(2,:),'Color','k','LineWidth',1);
s135 = plot(rot_spoke_135(1,:),rot_spoke_135(2,:),'Color','k','LineWidth',1);
shg
axis equal
axis([-0.2,0.2,-0.1,0.3])
Segway = [Sp.XData w.XData s.XData s45.XData s90.XData s135.XData;
          Sp.YData w.YData s.YData s45.YData s90.YData s135.YData];
Spc = size(segway_pendulum,2);
wc = size(wheel,2);
sc = size(spoke,2);
i = 1;
tic
t = toc*animspeed;
while t < tend
    z = deval(solution,t);
    phi = z(1); x = -z(2);
    Segway_shape = [rotate(Segway(:,1:Spc),phi) rotate(Segway(:,Spc+1:end),-x/r+phi)];
    Segway_shape = translate(Segway_shape,[x;0]);
    Sp.XData = Segway_shape(1,1:Spc);
    Sp.YData = Segway_shape(2,1:Spc);
    w.XData = Segway_shape(1,Spc+1:Spc+wc);
    w.YData = Segway_shape(2,Spc+1:Spc+wc);
    s.XData = Segway_shape(1,Spc+wc+1:Spc+wc+sc);
    s.YData = Segway_shape(2,Spc+wc+1:Spc+wc+sc);
    s45.XData = Segway_shape(1,Spc+wc+sc+1:Spc+wc+2*sc);
    s45.YData = Segway_shape(2,Spc+wc+sc+1:Spc+wc+2*sc);
    s90.XData = Segway_shape(1,Spc+wc+2*sc+1:Spc+wc+3*sc);
    s90.YData = Segway_shape(2,Spc+wc+2*sc+1:Spc+wc+3*sc);
    s135.XData = Segway_shape(1,Spc+wc+3*sc+1:Spc+wc+4*sc);
    s135.YData = Segway_shape(2,Spc+wc+3*sc+1:Spc+wc+4*sc);
    F(i) = getframe(gcf);
    drawnow
    axis([-2*L+x,2*L+x,-1.2*L,1.2*L])
    t = toc*animspeed;
    i = i+1;
end
hold off
if save
    filename = 'Segway.avi';
    writerObj = VideoWriter(filename);
    writerObj.FrameRate = 10;
    % set the seconds per image
    % open the video writer
    open(writerObj);
    % write the frames to the video
    for i=1:length(F)
        % convert the image to a frame
        frame = F(i) ;    
        writeVideo(writerObj, frame);
    end
    % close the writer object
    close(writerObj);
end