function [] = node_initialization(obj)
%node_initialization: initialize node
%{
% obj.node_position   3*N
% obj.node_position   3*N
% obj.node_rotation   3N*3

% obj.node_weight_id  N*num_nearestpts
% obj.node_weight     N*num_nearestpts
%}


% use matlab function to do downsample
ptc = pointCloud(obj.modelVertices');
ptc_down = pcdownsample(ptc,'gridAverage',obj.grid_size);   
obj.node_position = ptc_down.Location';
obj.num_nodes     = size(obj.node_position,2); 


% check downsample
%{
figure
plot3(obj.modelVertices(1,:),obj.modelVertices(2,:),obj.modelVertices(3,:),'b.', 'MarkerSize', 0.5);  hold on
plot3(obj.node_position(1,:),obj.node_position(2,:),obj.node_position(3,:),'k*');
axis equal
%}

% node connections
obj.num_weight = obj.num_nodes-1;     %for calculating weights of control_vertices (start from second one)
[obj.node_weight_id, ~ ] = updateWeight_knn(obj.node_position', obj.node_position', obj.num_weight);

obj.node_rotation  = repmat(eye(3),obj.num_nodes*3,1);   % deformation parameters
obj.node_translation = zeros(3,obj.num_nodes);           % notice: this is arranged differently for avoiding use transpose inside jacobian

end