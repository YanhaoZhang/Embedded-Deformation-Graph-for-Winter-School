function [] = control_pts_correspondence(obj)   
%main function to calculate correspondence
% previous commit:
% use function for re-calculating correspondence among gaussian-newton iteration
% this is an extention of my previous code

% output: controlpoints: before and after; relative pose
%         indexCorrespondence_observation2ModelContour_All: corresponding index of model contour
%         indexCorrespondence_observation2Model_All:        corresponding index of model
%         observation_EDGraph_All:                          controlvertices: after
%         CorrespondModelVertices_All:                      controlvertices: before

% the correspondence threshold affects a lot, especially for the ocludded part
% Yanhao 20190314


for k=1:obj.num_frames
    %% model 
    [vertex_proj] = camera_projection_model(obj.modelVertices, obj.Camera{k}.R, obj.Camera{k}.t, obj.Camera{k}.s);    % project model vertices to image frame
    [proj_contour, proj_contour_id] = calculate_projectionContour(vertex_proj', obj.alphaShape);  % calculate projection contour
    proj_contour_nv = LineNormals2D(proj_contour, obj.NormalVectorLines);                         % normal vector of model contour

    %% observation and normal vector
    obser = obj.Observation{k}.obs_p;
    obser_nv = obj.Observation{k}.obs_n;
    
    %% correspondence
    [index_obser2ModelContour,observation_after] = calculate_correspondence_observation2ModelContour(...
        obser,...
        obser_nv,...
        proj_contour, ...
        proj_contour_nv,...
        obj.theta_threshold, ...
        obj.dist_threshold);
    indexCorrespondence_observation2Model = proj_contour_id(index_obser2ModelContour,1);
    
    %% save output
    
    % control vertices
    obj.control_vertex_prior{k} = obj.modelVertices(:,indexCorrespondence_observation2Model);  % 3D position of observed vertices beform deformation, they are in model frame (before rotation)
    obj.control_vertex_after{k} = observation_after';                                          % observation used as control after 2D pixel coordinate
    
    
    %% corresponding ed nodes
    [obj.control_vertex_prior_weight_id{k}, obj.control_vertex_prior_weight{k} ] = updateWeight_knn(obj.control_vertex_prior{k}', obj.node_position', obj.num_nearestpts); 

    % other output
    obj.num_controlVertices(k) = size(obj.control_vertex_after{k},2);     % used in jacobian and F
end
obj.num_control_pts_all = sum(obj.num_controlVertices);         % all transformed points used for construct jacobian and F
end