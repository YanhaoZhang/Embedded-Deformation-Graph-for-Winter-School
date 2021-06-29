function [] = crossplane_vertex_transform(obj)
% transform the cross-section plane to the new position. Using the new
% position of each selected centerline3D



% calculate centerline tangent of pre-deformed centerline3D
[centerline3d_predef_tangent, ~] = tangent3D(obj.modelCenterline3D_predef', obj.NeighbourLines_skel3D);
modelCenterline3DTangent_predef = centerline3d_predef_tangent';

% calculate selected centerline and tangent 
selected_centerline3D        = obj.modelCenterline3D(:, obj.selected_skel_id);
selected_centerline3DTangent = obj.modelCenterline3DTangent(:, obj.selected_skel_id');  % tangent of selected 3D centerline
selected_centerline3D_predef = obj.modelCenterline3D_predef(:, obj.selected_skel_id);
selected_centerline3DTangent_predef = modelCenterline3DTangent_predef(:, obj.selected_skel_id');  % tangent of selected 3D centerline

% check selected centerline
%{
figure
plot3(obj.modelCenterline3D(1,:),obj.modelCenterline3D(2,:),obj.modelCenterline3D(3,:),'b.', 'MarkerSize', 1);  hold on
plot3(obj.modelCenterline3D_predef(1,:),obj.modelCenterline3D_predef(2,:),obj.modelCenterline3D_predef(3,:),'r.', 'MarkerSize', 1); hold on
plot3(selected_centerline3D(1,:),selected_centerline3D(2,:),selected_centerline3D(3,:),'k*');                   hold on
plot3(selected_centerline3D_predef(1,:),selected_centerline3D_predef(2,:),selected_centerline3D_predef(3,:),'k*');  hold on


quiver3(selected_centerline3D(1,:),selected_centerline3D(2,:),selected_centerline3D(3,:), ...
        selected_centerline3DTangent(1,:),selected_centerline3DTangent(2,:),selected_centerline3DTangent(3,:), 'k'); hold on
quiver3(selected_centerline3D_predef(1,:),selected_centerline3D_predef(2,:),selected_centerline3D_predef(3,:), ...
        selected_centerline3DTangent_predef(1,:),selected_centerline3DTangent_predef(2,:),selected_centerline3DTangent_predef(3,:), 'k'); hold on
axis equal
%}

%% transform vertex
obj.crosssection_vertex_after = cell(obj.num_selected_skel,1);   % store vertex new position
for k=1:obj.num_selected_skel
    z01 = selected_centerline3DTangent(:,k);
    z02 = selected_centerline3DTangent_predef(:,k);
    z10 = [0 0 1]';
    R01 = fcn_RotationFromTwoVectors(z10, z01); % rotation z0 = R01*z1
    R02 = fcn_RotationFromTwoVectors(z10, z02); % rotation z0 = R01*z1
    
    t_0_10 = selected_centerline3D(:,k);
    t_0_20 = selected_centerline3D_predef(:,k);  
    
    v_id = obj.crosssection_vertex_id{k};
    v_0_i0 = obj.modelVertices(:,v_id);       % vertex on current cross-section plane
    
    v_0_ii0 = R02*R01'*(v_0_i0 - t_0_10) + t_0_20;
    
    obj.crosssection_vertex_after{k} = v_0_ii0;
    

    % check
    %{
    plot3(selected_centerline3D(1,:),selected_centerline3D(2,:),selected_centerline3D(3,:),'b.');                   hold on
    plot3(selected_centerline3D_predef(1,:),selected_centerline3D_predef(2,:),selected_centerline3D_predef(3,:),'r.');  hold on
    plot3(t_0_10(1),t_0_10(2),t_0_10(3),'bo');   hold on
    plot3(t_0_20(1),t_0_20(2),t_0_20(3),'ro');   hold on
    plot3(v_0_i0(1,:),v_0_i0(2,:),v_0_i0(3,:),'b*');                   hold on
    plot3(v_0_ii0(1,:),v_0_ii0(2,:),v_0_ii0(3,:),'r*');                   hold on
    axis equal
    %}
    
end




end


% function [ weights_id, weights_normalized ] = transform_vertex(pts, pts_tang, pts2, pts2_tang, length_segment, z_lim)
% 
% 
% 
% end