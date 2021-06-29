function [] = crossplane_skel_downsample(obj)
% downsample 3D centerline


obj.selected_skel_id = 1:obj.downsample_step:obj.num_Centerline3D;

obj.num_selected_skel = length(obj.selected_skel_id);

% check downsample
%{
figure
plot3(obj.modelCenterline3D(1,:),obj.modelCenterline3D(2,:),obj.modelCenterline3D(3,:),'b.', 'MarkerSize', 0.5);  hold on
plot3(obj.modelCenterline3D(1,obj.selected_skel_id),obj.modelCenterline3D(2,obj.selected_skel_id),obj.modelCenterline3D(3,obj.selected_skel_id),'k*');
axis equal
%}




end