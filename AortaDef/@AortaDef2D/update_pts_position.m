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
        
        
        
        % todo you need to finish this part to update the vertices
        
    end
    points_new(:,i) = pts_tem;
end
end