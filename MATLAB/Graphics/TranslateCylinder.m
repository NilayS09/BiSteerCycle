function TranslateCylinder(Tvec,rod,i0)
    Tvec = reshape(Tvec,1,3);
    Tvec = [Tvec ;Tvec];
    rod.XData = i0(1:2,:) +  Tvec(:,1);
    rod.YData = i0(3:4,:) +  Tvec(:,2);
    rod.ZData = i0(5:6,:) +  Tvec(:,3);
end