function [] = crossplane_vertex_correspondence(obj)   
%crossplane_vertex_correspondence: main function to calculate
%corresponding vertex of each selected centerline points. 




% calculate centerline tangent
[centerline3d_mesh_tangent, ~] = tangent3D(obj.modelCenterline3D', obj.NeighbourLines_skel3D);
obj.modelCenterline3DTangent = centerline3d_mesh_tangent';


% calculate corresponding vertex on the cross-section plane
selected_centerline3D = obj.modelCenterline3D(:, obj.selected_skel_id);
selected_centerline3DTangent = obj.modelCenterline3DTangent(:, obj.selected_skel_id');  % tangent of selected 3D centerline

[obj.crosssection_vertex_id, ~ ] = ...
    updateWeight_centreline_unipts(selected_centerline3D, selected_centerline3DTangent, obj.modelVertices, obj.num_connect_pts2skel, obj.length_segment, obj.z_lim);



% check if there are any duplicate vertex id
% it does not metter if there are duplicate vertex, as edgraph treats it as
% a observation term. They must have similar, if not same weight. So it
% should be equivalent that I use the mean to do it. 

% crosssection_vertex_id_raw = [];
% for i=1:obj.num_selected_skel
%     crosssection_vertex_id_raw = [crosssection_vertex_id_raw, obj.crosssection_vertex_id{i}];
%     
% end
% 
% if length(crosssection_vertex_id_raw) == length(unique(crosssection_vertex_id_raw))
%     error('duplicate correspond vertex')
% end


end