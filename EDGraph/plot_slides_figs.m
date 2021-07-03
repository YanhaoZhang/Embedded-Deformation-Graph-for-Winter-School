clear
close all

% load data
load('./data/bun_zipper.mat')

%%
%{
ctrl_pts_red = model.vertices(ctrl_pts_id_red,:);
ctrl_pts_blue = model.vertices(ctrl_pts_id_blue,:);

obser.ctrl_pts_prior = [ctrl_pts_red; ctrl_pts_blue]';
ctrl_tmp = ctrl_pts_red + [.01 .05 0.05];
obser.ctrl_pts_after = [ctrl_tmp; ctrl_pts_blue]';

ptc = pointCloud(model.vertices);
ptc_down = pcdownsample(ptc,'gridAverage',0.02);   
node_position = ptc_down.Location;


figure
patch(model,'FaceColor',    [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'facealpha',       0.6, ...
         'AmbientStrength', 0.15);
camlight('headlight');
material('dull');
axis equal
hold on
plot3(node_position(:,1),node_position(:,2),node_position(:,3),'ko'); 

% draw axis
hold on
axisl = 0.01;
rotation = eye(3);
for i=1:size(node_position,1)
positionI = node_position(i,:)';
xdir = positionI + axisl*rotation(:, 1);      %This is just the point of the local x-axis times 5
xdir = [positionI, xdir];
ydir = positionI + axisl*rotation(:, 2);
ydir = [positionI, ydir];
zdir = positionI + axisl*rotation(:, 3);
zdir = [positionI, zdir];
plot3(xdir(1,:), xdir(2,:), xdir(3,:), 'Color', 'red', 'LineWidth',3); hold on;
plot3(ydir(1,:), ydir(2,:), ydir(3,:), 'Color', 'blue', 'LineWidth',3); hold on;
plot3(zdir(1,:), zdir(2,:), zdir(3,:), 'Color', 'green', 'LineWidth',3); hold on;
end
%}

%% 
ctrl_pts_red = model.vertices(ctrl_pts_id_red,:);
ctrl_pts_blue = model.vertices(ctrl_pts_id_blue,:);

obser.ctrl_pts_prior = [ctrl_pts_red; ctrl_pts_blue]';
ctrl_tmp = ctrl_pts_red + [.05 .05 0.05];
obser.ctrl_pts_after = [ctrl_tmp; ctrl_pts_blue]';

% check input data
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
plot3(ctrl_pts_blue(:,1),ctrl_pts_blue(:,2),ctrl_pts_blue(:,3),'b.'); 

plot3(ctrl_tmp(:,1),ctrl_tmp(:,2),ctrl_tmp(:,3),'r.'); 
%}

edgraph3d = EDGraph3D(model, obser);
edgraph3d = saveToStruct(edgraph3d);


% check result
model_rst.faces = model.faces;
model_rst.vertices = edgraph3d.modelVertices';

figure
patch(model_rst,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
camlight('headlight');
material('dull');
axis equal
hold on
plot3(ctrl_pts_red(:,1),ctrl_pts_red(:,2),ctrl_pts_red(:,3),'r.'); 
plot3(ctrl_pts_blue(:,1),ctrl_pts_blue(:,2),ctrl_pts_blue(:,3),'b.'); 

plot3(ctrl_tmp(:,1),ctrl_tmp(:,2),ctrl_tmp(:,3),'r.'); 
