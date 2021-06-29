function [] = Gauss_Newton_Optimization(obj)        
% main function for predeform model




%% corresponding model vertex from centerline

% down-sample 3d centerline (do not use all points)
crossplane_skel_downsample(obj);

% calculate cross section plane and corresponding model vertex
crossplane_vertex_correspondence(obj);

% calculate new position of cross section plane
crossplane_vertex_transform(obj);


%% pre-deform model based on corresponding cross-section vertex

% re-write data and calculate correspond EDNodes of each cross-section plane. 
control_vertex_correspondence(obj);

% calculate jacobian and error
calculate_Error(obj);
calculate_Jacobian(obj);

% initialize state variable x
[x] = x_initialization(obj);

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
    obj.iter_times = iter_times;

    % calculate jacobian and error for next step
    calculate_Error(obj);
    calculate_Jacobian(obj);
    min_FX_new = obj.Error_whole'*obj.Error_whole;     % cost function
    
    % check next iteration
    aa_dx = dx'*dx ;
    fprintf('iter, dx, Fx_old, min_FX_new:  %d,  %f,  %f,  %f\n', iter_times, full(aa_dx), full(min_FX_old), full(min_FX_new))
    if (aa_dx > obj.StopIterationCriteria && abs(min_FX_new-min_FX_old)>obj.StopIterationCriteria && iter_times<obj.iter )   % dx, d_error, iteration_times
        NeedOptimization = true;
    else
        NeedOptimization = false;               % all abs( increment ) is smaller than thredhood
    end
    
    
    %% update model for debug
    %{
    figure
    [modelVertices] = update_pts_position(obj, obj.modelVertices, obj.num_nearestpts);
    selected_centerline3D_predef = obj.modelCenterline3D_predef(:, obj.selected_skel_id);
    
    plot3(selected_centerline3D_predef(1,:),selected_centerline3D_predef(2,:),selected_centerline3D_predef(3,:),'r.');  hold on
    plot3(modelVertices(1,:),modelVertices(2,:),modelVertices(3,:),'b.', 'MarkerSize', 0.5);  hold on
    for k=1:obj.num_selected_skel
        vertice_after   = obj.crosssection_vertex_after{k};
        plot3(vertice_after(1,:),vertice_after(2,:),vertice_after(3,:),'k*');  hold on
    end
    axis equal
    hold off
    %}
end
% update model centerline
[obj.modelVertices] = update_pts_position(obj, obj.modelVertices, obj.num_nearestpts);
end



