function [points_new] = update_pts_position(obj, points, num_nearestpts)
%update_model_vertex: update model vertices accouding to new EDNode rotation and translation






% deside which to be updated
% num_nearestpts = obj.num_nearestpts_skel;
% points = obj.modelVertices;                   % start from the initial instead of updated

[pts_weights_id, pts_weights ] = updateWeight_knn(points', obj.node_position', num_nearestpts); 
num_pts = size(points,2);
points_new = zeros(3,num_pts);

for i=1:num_pts
    pts_i  = points(:,i);
    weight_i  = pts_weights(i,:);
    weight_id = pts_weights_id(i,:);
    pts_tem = zeros(3,1);
    for j=1:num_nearestpts
        
        node_id = weight_id(j);
        
        node_R = obj.node_rotation(3*node_id-2:3*node_id,:);
        node_p = obj.node_position(:,node_id);
        node_t = obj.node_translation(:,node_id);
        mapped_pts = node_R * (pts_i - node_p) + node_p + node_t;   %%%Yanhao20181225: Eq. (1)
        pts_tem = pts_tem + weight_i(j) * mapped_pts;
    end
    points_new(:,i) = pts_tem;
end
end


%% previous version:
% 
% 
% 
% vertices.positions  = modelVertices_org;                                                                          %%%Yanhao20181225: 这里的pointcloud不是node 是原始点云
% modelVertices_new      = modelVertices_org;
% vertices.weights    = UpdateWeights(vertices.positions,ED_Parameter.node.positions,ED_Parameter.num_nearestpts);
% for i = 1 : size(vertices.positions,1)
%     vertices_tmp = [0,0,0];
%     for j = 1 : ED_Parameter.num_nearestpts
%         vertices_pts = vertices.positions(i,:);
%         weight_tmp = vertices.weights(i,2*j);
%         mapped_point = Map_Points( vertices_pts, ED_Parameter.node, vertices.weights(i,2*j-1));           %%%Yanhao20181225: 更新所有点pointcloud Eq. (1)
%         vertices_tmp = vertices_tmp + weight_tmp * mapped_point;                                          %                  Eq. (2)
%     end
%     modelVertices_new(i,:) = vertices_tmp;
% end
% 
% modelVertices_org = modelVertices_new;
% 
% 
% function [ mapped_vertices ] = Map_Points( vertice, node, node_index )
% %MAP_POINTS Summary of this function goes here
% %   map a point according to a given node
% 
% %%%Yanhao20181225: 根据Eq. (1)计算pointcloud没个点新的位置．　实际上是Eq. (2), 不过加权和写在外面了.
% 
% 
% 
% node_positions = node.positions(node_index,:);                                 %%%Yanhao20181225: 根据node_index找到对应的position rotation translation
% node_rotation_matrix = node.rotation_matrix(3*node_index-2:3*node_index,:);
% node_transformation_matrix = node.transformation_matrix(node_index,:);
% 
% mapped_vertices = node_rotation_matrix * (vertice' - node_positions') + node_positions' + node_transformation_matrix';   %%%Yanhao20181225: Eq. (1)
% mapped_vertices = mapped_vertices';
% end
