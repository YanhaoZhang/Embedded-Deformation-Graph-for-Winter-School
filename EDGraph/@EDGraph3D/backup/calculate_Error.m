function [] = calculate_Error(obj)
%calculate_Error: calculate jacobian


%% initialize residual function
dim_obs = 3;                   % dim of obs constraint is 1
% dim_len = 1;                   % dim of len constration is 1
% num_len = sum(sum(obj.Link));  % num of length term

obj.Error_whole = ones(obj.num_nodes*(6+3*obj.num_connection) ...        % ednode
                       + dim_obs*obj.num_crosssectionVertices_all, ... % observation term
                       1);

% sigma_rot = obj.sigma_rot/sqrt(6*obj.num_nodes);
% sigma_reg = obj.sigma_reg/sqrt(3*obj.num_nodes*obj.num_connection);
% sigma_obs = obj.sigma_obs/sqrt(dim_obs*obj.num_crosssectionVertices_all);
sigma_rot = obj.sigma_rot;
sigma_reg = obj.sigma_reg;
sigma_obs = obj.sigma_obs;


%% dealing with E_rot and E_reg    'embedded deformation for shape manipulation' Eq 5 6 7
for j = 1 : obj.num_nodes
    node_Rj = obj.node_rotation(3*j-2:3*j,:);
    node_gj = obj.node_position(:,j);
    node_tj = obj.node_translation(:,j);
    weight_id = obj.node_weight_id(j,:);
    f_x_node = zeros(6 + 3 * obj.num_connection,1);       % f_x_node is the temperary f_x for each node
    
    % E_rot
    f_x_node(1) = sigma_rot * (node_Rj(1) * node_Rj(4) + node_Rj(2) * node_Rj(5) + node_Rj(3) * node_Rj(6));     %%%Yanhao20181225: 前６个是rotation的constraint, 后面3n个是translation的constraint
    f_x_node(2) = sigma_rot * (node_Rj(1) * node_Rj(7) + node_Rj(2) * node_Rj(8) + node_Rj(3) * node_Rj(9));     %Yanhao20190101: rotation按竖着的顺序排列的
    f_x_node(3) = sigma_rot * (node_Rj(4) * node_Rj(7) + node_Rj(5) * node_Rj(8) + node_Rj(6) * node_Rj(9));
    f_x_node(4) = sigma_rot * (node_Rj(1)^2 + node_Rj(2)^2 + node_Rj(3)^2 - 1);
    f_x_node(5) = sigma_rot * (node_Rj(4)^2 + node_Rj(5)^2 + node_Rj(6)^2 - 1);
    f_x_node(6) = sigma_rot * (node_Rj(7)^2 + node_Rj(8)^2 + node_Rj(9)^2 - 1);
    
    for i = 1:obj.num_connection
        node_id = weight_id(i + 1);    % j+1 不是 j. j相当于原来的node
        node_gi = obj.node_position(:,node_id);
        node_ti = obj.node_translation(:,node_id);
        f_x_node(6+3*(i-1)+1:6+3*i) = ...                                                       %%%Yanhao20181226: weight怎么不见了？ embedded graph文章中说weight=1
            sigma_reg * (node_Rj * (node_gi - node_gj) + node_gj + node_tj - (node_gi + node_ti));
    end
    obj.Error_whole((6+3*obj.num_connection)*(j-1)+1:(6+3*obj.num_connection)*j) = f_x_node;    %%%Yanhao20181225: 对于每个node 都有6个E_rot的f 3n个E_reg
end


%% observation: cross-section control points
position_x     = obj.num_nodes*(6+3*obj.num_connection);
observation_ii = 0;    % index of observation in f_x

for k=1:obj.num_selected_skel
    vertice_prior   = obj.crosssection_vertex_prior{k};
    weight_prior    = obj.crosssection_vertex_prior_weight{k};
    weight_prior_id = obj.crosssection_vertex_prior_weight_id{k};
    vertice_after   = obj.crosssection_vertex_after{k};

    for i=1:obj.num_crosssectionVertices(k)
        % calculate nodes before transform using model2camera pose, which means the point_rot_trans is in model frame
        vertex_prior_I = vertice_prior(:,i);
        weight_prior_I = weight_prior(i,:);
        weight_id      = weight_prior_id(i,:);
        
        vertex_mapped = zeros(3,1);
        for j = 1 : obj.num_nearestpts
            % point_rot_trans before transform, which means the point_rot_trans is in model frame
            node_id = weight_id(j);
            node_Rj = obj.node_rotation(3*node_id-2:3*node_id,:);
            node_gj = obj.node_position(:,node_id);
            node_tj = obj.node_translation(:,node_id);
            node_wj = weight_prior_I(j);

            % 'embedded deformation for shape manipulation' Eq 2
            vertex_mapped = vertex_mapped + node_wj * (node_Rj * (vertex_prior_I - node_gj) + node_gj + node_tj);
        end
        
        % calculate error

        tem_diff = (vertex_mapped - vertice_after(:,i));           % 所有的误差
        
        observation_ii = observation_ii + 1;
        obj.Error_whole(position_x+dim_obs*(observation_ii-1)+1:position_x+dim_obs*observation_ii,1) = sigma_obs * tem_diff;         %%%Yanhao20181225:　观测误差E_d default只在z方向做了Deform
    end
end





obj.Error_whole = sparse(obj.Error_whole);
end