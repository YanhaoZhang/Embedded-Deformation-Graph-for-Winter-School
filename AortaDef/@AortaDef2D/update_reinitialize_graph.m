function [] = update_reinitialize_graph(obj)
%update_node_parameter: only update rotation and translation, not position of nodes
% x: state variable
% NOITCE: the arrange of node_translation is different from node_rotation. This is for avoiding transpose when calculating Jacobian and error


%% update node position
[obj.node_position] = update_pts_position(obj, obj.node_position, obj.num_nearestpts);

%% ed parameters
obj.node_rotation  = repmat(eye(3),obj.num_nodes*3,1);  
obj.node_translation = zeros(3,obj.num_nodes);     

end