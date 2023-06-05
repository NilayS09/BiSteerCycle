function RotateCylinder(R,rod,i0)
    coords = [reshape(i0(1:2,:),42,1)';
              reshape(i0(3:4,:),42,1)';
              reshape(i0(5:6,:),42,1)'];
    coords = R*coords;
    X = coords(1,:);
    Y = coords(2,:);
    Z = coords(3,:);
    
    rod.XData = reshape(X',2,21);
    rod.YData = reshape(Y',2,21);
    rod.ZData = reshape(Z',2,21);
end