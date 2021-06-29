clear
close all

% load data
load('./data/bun_zipper.mat')


ctrl_pts_red = model.vertices(ctrl_pts_id_red,:);
ctrl_pts_blue = model.vertices(ctrl_pts_id_blue,:);

obser.ctrl_pts_prior = [ctrl_pts_red; ctrl_pts_blue]';
ctrl_tmp = ctrl_pts_red + [.01 .05 0.05];
obser.ctrl_pts_after = [ctrl_tmp; ctrl_pts_blue]';



% check data
%{
figure
patch(model,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
camlight('headlight');
material('dull');
axis equal
hold on
plot3(ctrl_pts_red(:,1),ctrl_pts_red(:,2),ctrl_pts_red(:,3),'r.'); 
plot3(ctrl_pts_blue(:,1),ctrl_pts_blue(:,2),ctrl_pts_blue(:,3),'r.'); 
%}




edgraph3d = EDGraph3D(model, obser);
edgraph3d = saveToStruct(edgraph3d);

% check result
%{
model_rst.faces = model.faces;
model_rst.vertices = edgraph3d.modelVertices';


figure
patch(model,'FaceColor',       [0.0 0.0 0.6], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15); 
hold on
plot3(obser.ctrl_pts_prior(1,:),obser.ctrl_pts_prior(2,:),obser.ctrl_pts_prior(3,:),'k.'); 
hold on;
patch(model_rst,'FaceColor',       [0.6 0.0 0.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
camlight('headlight');
material('dull');
hold on
plot3(obser.ctrl_pts_after(1,:),obser.ctrl_pts_after(2,:),obser.ctrl_pts_after(3,:),'g.');
%}
