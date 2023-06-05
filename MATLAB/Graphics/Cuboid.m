function box = Cuboid(w,d,h,C)

    X = [-w/2 -w/2  w/2  w/2 -w/2 -w/2  w/2  w/2]';
    Y = [-d/2  d/2  d/2 -d/2 -d/2  d/2  d/2 -d/2]';
    Z = [-h/2 -h/2 -h/2 -h/2  h/2  h/2  h/2  h/2]';
    
    box_vertices = [X,Y,Z];
    box_faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

    box = patch('Vertices',box_vertices,'Faces',box_faces,'FaceColor',C);

end