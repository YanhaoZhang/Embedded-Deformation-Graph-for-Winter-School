function [] = update_node_parameter(obj,x)
%update_node_parameter: only update rotation and translation, not position of nodes
% x: state variable
% NOITCE: the arrange of node_translation is different from node_rotation. This is for avoiding transpose when calculating Jacobian and error



for i=1:obj.num_nodes
    
    obj.node_rotation((i-1)*3+1:(i-1)*3+3,:) = reshape(x((i-1)*12+1:(i-1)*12+9),3,3);    % reshape orders array from column: aaa = 1:9; reshape(aaa,3,3)
    obj.node_translation(:,i)                = x((i-1)*12+10:(i-1)*12+12);
end

end