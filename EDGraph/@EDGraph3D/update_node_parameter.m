function [] = update_node_parameter(obj,x)
%update_node_parameter: only update rotation and translation, not position of nodes
% x: state variable
% NOITCE: the arrange of node_translation is different from node_rotation. This is for avoiding transpose when calculating Jacobian and error



for i=1:obj.num_nodes
    
    obj.node_rotation((i-1)*3+1:(i-1)*3+3,:) = reshape(x((i-1)*12+1:(i-1)*12+9),3,3);    % reshape orders array from column: aaa = 1:9; reshape(aaa,3,3)
    obj.node_translation(:,i)                = x((i-1)*12+10:(i-1)*12+12);
end

%% previous version:
%     
%     x_optimum = zeros(row,col);
% for i = 1 : num_nodes
%     x_optimum(3*(i-1)+1,1) = x((i-1)*12+1);
%     x_optimum(3*(i-1)+2,1) = x((i-1)*12+2);
%     x_optimum(3*(i-1)+3,1) = x((i-1)*12+3);
%     x_optimum(3*(i-1)+1,2) = x((i-1)*12+4);
%     x_optimum(3*(i-1)+2,2) = x((i-1)*12+5);
%     x_optimum(3*(i-1)+3,2) = x((i-1)*12+6);
%     x_optimum(3*(i-1)+1,3) = x((i-1)*12+7);
%     x_optimum(3*(i-1)+2,3) = x((i-1)*12+8);
%     x_optimum(3*(i-1)+3,3) = x((i-1)*12+9);
%     x_optimum(end - num_nodes + i,1) = x((i-1)*12+10);
%     x_optimum(end - num_nodes + i,2) = x((i-1)*12+11);
%     x_optimum(end - num_nodes + i,3) = x((i-1)*12+12);
% end
%     
% ED_Parameter.node.rotation_matrix       = x_optimum(1: 3*ED_Parameter.num_nodes,:);                        %%%Yanhao20181225: update优化结果
% ED_Parameter.node.transformation_matrix = x_optimum(3*ED_Parameter.num_nodes+1: end,:);
end