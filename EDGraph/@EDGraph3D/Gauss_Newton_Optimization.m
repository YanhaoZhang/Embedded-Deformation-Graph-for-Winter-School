function [] = Gauss_Newton_Optimization(obj)        
% main function for predeform model







% control_pts_correspondence(obj);

% calculate jacobian and error
calculate_Error(obj);
calculate_Jacobian(obj);


% initialize state variable x
[x] = x_initialization(obj);


ifdebug = true;
if ifdebug
%{
figure
[modelVertices] = update_pts_position(obj, obj.modelVertices, obj.num_nearestpts);
plot3(modelVertices(1,:), modelVertices(2,:), modelVertices(3,:), 'b.'); hold on
plot3(obj.control_pts_prior(1,:), obj.control_pts_prior(2,:), obj.control_pts_prior(3,:), 'k.'); hold on
plot3(obj.control_pts_after(1,:), obj.control_pts_after(2,:), obj.control_pts_after(3,:), 'r.'); hold on
%}
end
    

% perform gauss-newton
NeedOptimization = true;                                           % flag controls iteration
iter_times     = 0;                                                % iteration times
while(NeedOptimization)
    min_FX_old = obj.Error_whole'*obj.Error_whole;
    
    % dx
    % NOTICE: sign '-1'
    dx=  -(obj.Jacobian_whole'*obj.Jacobian_whole)\(obj.Jacobian_whole'*obj.Error_whole);  % this is the incremental vector

    % update ED_nodes
    x = x + dx;
    update_node_parameter(obj,x);             % only update rotation and translation, not position of nodes
    
    % iteration time
    iter_times = iter_times + 1;

    % calculate jacobian and error for next step
    calculate_Error(obj);
    calculate_Jacobian(obj);
    min_FX_new = obj.Error_whole'*obj.Error_whole;     % cost function
    
    % check next iteration
    aa_dx = dx'*dx ;
    if (aa_dx > obj.StopIterationCriteria && abs(min_FX_new-min_FX_old)>obj.StopIterationCriteria && iter_times<obj.iter )   % dx, d_error, iteration_times
        NeedOptimization = true;
    else
        NeedOptimization = false;               % all abs( increment ) is smaller than thredhood
    end
    
    if ifdebug
    fprintf('iter, dx, Fx_old, min_FX_new:  %d,  %f,  %f,  %f\n', iter_times, full(aa_dx), full(min_FX_old), full(min_FX_new))
    
    % check model in each iteration
    %{
    figure
    [modelVertices] = update_pts_position(obj, obj.modelVertices, obj.num_nearestpts);
    plot3(modelVertices(1,:), modelVertices(2,:), modelVertices(3,:), 'b.'); hold on
    plot3(obj.control_pts_prior(1,:), obj.control_pts_prior(2,:), obj.control_pts_prior(3,:), 'k.'); hold on
    plot3(obj.control_pts_after(1,:), obj.control_pts_after(2,:), obj.control_pts_after(3,:), 'r.'); hold off
    %}
    end
end
% update model centerline
[obj.modelVertices] = update_pts_position(obj, obj.modelVertices, obj.num_nearestpts);
end



