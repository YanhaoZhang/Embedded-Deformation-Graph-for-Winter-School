addpath('../utils/')

[vertex,face, color] = read_ply('./bun_zipper_colored.ply');


model.vertices = vertex;
model.faces    = face;

ctrl_pts_id_red = find(color(:,1) == 255 & color(:,2) == 0 & color(:,3) == 0 );   % red point is static
ctrl_pts_id_blue = find(color(:,1) == 0 & color(:,2) == 0 & color(:,3) == 255 );   % red point is static


figure
patch(model,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
camlight('headlight');
material('dull');
axis equal
hold on
plot3(model.vertices(ctrl_pts_id_red,1),model.vertices(ctrl_pts_id_red,2),model.vertices(ctrl_pts_id_red,3),'r.'); 
plot3(model.vertices(ctrl_pts_id_blue,1),model.vertices(ctrl_pts_id_blue,2),model.vertices(ctrl_pts_id_blue,3),'b.'); 

save('bun_zipper', 'model', 'ctrl_pts_id_red', 'ctrl_pts_id_blue')