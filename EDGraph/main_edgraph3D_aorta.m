clear
close all

addpath('utils')

%% load segmented data
[vertex,face, color] = read_ply('./data/aorta3D.ply');


model.vertices = vertex;
model.faces    = face;

ctrl_pts_id_red = find(color(:,1) == 255 & color(:,2) == 0 & color(:,3) == 0 );   % red point is static
ctrl_pts_id_blue = find(color(:,1) == 0 & color(:,2) == 0 & color(:,3) == 255 );   % red point is static

% check input data
subplot(1,3,1)
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
xlabel('X')
ylabel('Y')
zlabel('Z')
view(6,12)




%% select control points
ctrl_pts_red = model.vertices(ctrl_pts_id_red,:);
ctrl_pts_blue = model.vertices(ctrl_pts_id_blue,:);
obser.ctrl_pts_prior = [ctrl_pts_red; ctrl_pts_blue]';
ctrl_tmp = ctrl_pts_red + [0 0 20];
obser.ctrl_pts_after = [ctrl_tmp; ctrl_pts_blue]';

% check control points
subplot(1,3,2)
patch(model,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
camlight('headlight');
material('dull');
axis equal
hold on
plot3(ctrl_tmp(:,1),ctrl_tmp(:,2),ctrl_tmp(:,3),'r.'); 
plot3(ctrl_pts_blue(:,1),ctrl_pts_blue(:,2),ctrl_pts_blue(:,3),'b.'); 
xlabel('X')
ylabel('Y')
zlabel('Z')
view(6,12)



%% perform ed graph

edgraph3d = EDGraph3D(model, obser);
edgraph3d = saveToStruct(edgraph3d);

% check result
model_rst.faces = model.faces;
model_rst.vertices = edgraph3d.modelVertices';

subplot(1,3,3)
patch(model_rst,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
camlight('headlight');
material('dull');
axis equal
hold on
plot3(model_rst.vertices(ctrl_pts_id_red,1),model_rst.vertices(ctrl_pts_id_red,2),model_rst.vertices(ctrl_pts_id_red,3),'r.');  hold on
plot3(model_rst.vertices(ctrl_pts_id_blue,1),model_rst.vertices(ctrl_pts_id_blue,2),model_rst.vertices(ctrl_pts_id_blue,3),'b.'); 
xlabel('X')
ylabel('Y')
zlabel('Z')
view(6,12)


