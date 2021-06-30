function [] = control_pts_initialization(obj)
%initialize control points





% initialize control vertices:
obj.control_vertex_prior = cell(1,obj.num_frames);            % cell of 1*num_frames
obj.control_vertex_prior_weight = cell(1,obj.num_frames);     % cell of 1*num_frames
obj.control_vertex_prior_weight_id = cell(1,obj.num_frames);  % cell of 1*num_frames
obj.control_vertex_after = cell(1,obj.num_frames);
obj.num_controlVertices = zeros(1,obj.num_frames);
end