function [] = Gauss_Newton_Optimization(obj)        
% main function of gaussian newton iteration


% calculate initial correspondence: vertex-pixel
control_pts_correspondence(obj);   

% calculate jacobian and error
calculate_Error(obj);
calculate_Jacobian(obj);

ifdebug = 0;
if ifdebug
fig = figure;
show_result(obj, fig, 1);
end

 
% perform GN
NeedOptimization = true;      % flag controls iteration
iter_times     = 0;           % iteration times
while(NeedOptimization)
    min_FX_old = obj.Error_whole'*obj.Error_whole;
    
    % NOTICE: sign '-1'
    dx=  -(obj.Jacobian_whole'*obj.Jacobian_whole)\(obj.Jacobian_whole'*obj.Error_whole);  % this is the incremental vector
    
    % initialize state variable x
    [x] = x_initialization(obj);
    x = x + dx;

    % update ED_nodes
    update_node_parameter(obj,x);             % only update rotation and translation, not position of nodes
    
    % update model vertices
    [obj.modelVertices] = update_pts_position(obj, obj.modelVertices, obj.num_nearestpts);
    
    % todo you can recalculate correspondence here. Don't forget to
    % reinitialize the graph
    

    
    % iteration times update
    iter_times = iter_times + 1;

    % calculate jacobian and error for next step
    calculate_Error(obj);
    calculate_Jacobian(obj);
    min_FX_new = obj.Error_whole'*obj.Error_whole;  
    
    % check if stop iteration
    aa_dx = dx'*dx ;
    fprintf('iter, dx, Fx_old, min_FX_new:  %d,  %f,  %f,  %f\n', iter_times, full(aa_dx), full(min_FX_old), full(min_FX_new))
    if (aa_dx > obj.StopIterationCriteria && abs(min_FX_new-min_FX_old)>obj.StopIterationCriteria && iter_times<obj.iter )   % dx, d_error, iteration_times
        NeedOptimization = true;
    else
        NeedOptimization = false;               % all abs( increment ) is smaller than thredhood
    end
    
    if ifdebug
        show_result(obj, fig, 4);
    end
end
end





%% show some result for debug
function [] = show_result(obj, fig, k3D)


% show 3D model and ground truth
subplot(2,3,k3D)
plot3(obj.modelVertices(1,:),obj.modelVertices(2,:),obj.modelVertices(3,:),'b.', 'MarkerSize', 0.5); hold off

% show 3D projection
for k=1:2
    [vertex_proj] = camera_projection_model(obj.modelVertices, obj.Camera{k}.R, obj.Camera{k}.t, obj.Camera{k}.s);    % project model vertices to image frame
    [proj_contour, ~] = calculate_projectionContour(vertex_proj', obj.alphaShape);                      % calculate projection contour
    obser = obj.Observation{k}.obs_p;
    
    subplot(2,3,k3D+k)
    plot(obser(:,1),obser(:,2),'r.','Markersize',1); hold on;
    plot(proj_contour(:,1),proj_contour(:,2),'b.','Markersize',1); hold off
    axis equal
    title(['frame: ', num2str(k)])
end


end