close all
clear

% load data
load('./data/Camera.mat')
load('./data/model.mat')
load('./data/model_gt.mat')
load('./data/Observation.mat')





% reconstruct deformation
aorta_rst = AortaDef2D(model, Camera, Observation);
aorta_rst = saveToStruct(aorta_rst);


% check result
model_rst.vertices = aorta_rst.modelVertices';
model_rst.faces = model.faces;
TR = triangulation(model_gt.faces,model_gt.vertices);
normal_gt = vertexNormal(TR);

[ ~,meanError,~,error ] = computeError_result2groundtruth( model_rst.vertices, model_gt.vertices, normal_gt);


figure
patch(model_rst,'facecolor',[0 0 1],'facealpha',0.6,'edgecolor','none');
patch(model_gt,'facecolor',[0 1 0],'facealpha',0.6,'edgecolor','none');
camlight
legend('result','ground truth')
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal