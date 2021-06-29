function [] = control_pts_correspondence(obj, Obser)   
%calculate correspondence of control points. The original EDGraph assumes
%the correspondence is given. 




% control points
obj.control_pts_prior = Obser.ctrl_pts_prior;           
obj.control_pts_after = Obser.ctrl_pts_after; 

% corresponding ednode of each control points
[obj.control_pts_prior_weight_id, obj.control_pts_prior_weight ] = updateWeight_knn(obj.control_pts_prior', obj.node_position', obj.num_nearestpts);
obj.num_control_pts_prior = size(obj.control_pts_prior,2);

end