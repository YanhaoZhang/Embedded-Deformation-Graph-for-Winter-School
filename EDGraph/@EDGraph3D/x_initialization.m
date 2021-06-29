function [x] = x_initialization(obj)
% initialize state variable



% x = zeros(12*obj.num_nodes,1);
% for i=1:obj.num_nodes
%     x((i-1)*12+1:(i-1)*12+9) = reshape(obj.node_rotation((i-1)*3+1:(i-1)*3+3,:),9,1);
%     x((i-1)*12+10:(i-1)*12+12) = obj.node_translation(:,i);
% end


x = zeros(12*obj.num_nodes,1);
for i=1:obj.num_nodes
    x((i-1)*12+1:(i-1)*12+9) = reshape(eye(3),9,1);
%     x((i-1)*12+10:(i-1)*12+12) = obj.node_translation(:,i);
end

end