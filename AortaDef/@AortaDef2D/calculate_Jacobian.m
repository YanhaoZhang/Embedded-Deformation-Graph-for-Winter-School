function [] = calculate_Jacobian(obj)
%calculate_Jacobian: do not use sparse matrix. Need to be modified later



%% initialize
dim_obs = 2;                   % dim of obs constraint is 1
obj.Jacobian_whole = zeros(obj.num_nodes*(6+3*obj.num_connection) ...  % ednode
                   + dim_obs*obj.num_control_pts_all, ...   % observation term
                   obj.num_nodes * 12);
               

% sigma_rot = obj.sigma_rot/sqrt(6*obj.num_nodes);
% sigma_reg = obj.sigma_reg/sqrt(3*obj.num_nodes*obj.num_connection);
% sigma_obs = obj.sigma_obs/sqrt(dim_obs*obj.num_crosssectionVertices_all);
sigma_rot = obj.sigma_rot;
sigma_reg = obj.sigma_reg;
sigma_obs = obj.sigma_obs;



%% E_rot 'embedded deformation for shape manipulation' Eq 5 6 7
J_x_node = zeros(6 + 3 * obj.num_connection,   12);
for j = 1 : obj.num_nodes
    node_Rj = obj.node_rotation(3*j-2:3*j,:);
    node_gj = obj.node_position(:,j);
%     node_tj = obj.node_translation(:,i);
    weight_id = obj.node_weight_id(j,:);
    
    position_x = (j-1) * (6+3*obj.num_connection) + 1;
    position_y = (j-1) * 12 + 1;
    position_xx = j * (6+3*obj.num_connection);
    position_yy = j * 12;

    J_x_node(1,:) = sigma_rot * [node_Rj(4),node_Rj(5),node_Rj(6),node_Rj(1),node_Rj(2),node_Rj(3),0,0,0,0,0,0];  
    J_x_node(2,:) = sigma_rot * [node_Rj(7),node_Rj(8),node_Rj(9),0,0,0,node_Rj(1),node_Rj(2),node_Rj(3),0,0,0];
    J_x_node(3,:) = sigma_rot * [0,0,0,node_Rj(7),node_Rj(8),node_Rj(9),node_Rj(4),node_Rj(5),node_Rj(6),0,0,0];
    J_x_node(4,:) = sigma_rot * [2*node_Rj(1),2*node_Rj(2),2*node_Rj(3),0,0,0,0,0,0,0,0,0];
    J_x_node(5,:) = sigma_rot * [0,0,0,2*node_Rj(4),2*node_Rj(5),2*node_Rj(6),0,0,0,0,0,0];
    J_x_node(6,:) = sigma_rot * [0,0,0,0,0,0,2*node_Rj(7),2*node_Rj(8),2*node_Rj(9),0,0,0];

    
    %% E_reg: refer to my note: embedded deformation graph
    for i = 1:obj.num_connection
        node_id = weight_id(i+1);                 
        node_gi = obj.node_position(:,node_id);
%         node_tk = obj.node_translation(:,node_id);
        d_t = node_gi - node_gj;
        J_x_node(6+3*(i-1)+1:6+3*i,:) = ...                                                
            sigma_reg * [d_t(1)*eye(3,3),d_t(2)*eye(3,3),d_t(3)*eye(3,3),eye(3,3)];  

        % Ri and ti:
        potision_k = node_id * 12 - 2;
        obj.Jacobian_whole(position_x+6+3*(i-1):position_x+6+3*i-1,potision_k:potision_k+2) = sigma_reg * (-1)*eye(3,3); 

    end
    obj.Jacobian_whole(position_x:position_xx,position_y:position_yy) = J_x_node;
    
end


%% dealing with observation
position_x     = obj.num_nodes*(6+3*obj.num_connection);
observation_ii = 0;   % index of observation in f_x
for k=1:obj.num_frames
    vertice_prior  = obj.control_vertex_prior{k};
    weight_prior = obj.control_vertex_prior_weight{k};
    weight_prior_id = obj.control_vertex_prior_weight_id{k};

    for i=1:obj.num_controlVertices(k)
        %% calculate nodes before transform using model2camera pose, which means the point_rot_trans is in model frame
        vertex_prior_I  = vertice_prior(:,i);
        weight_prior_I  = weight_prior(i,:);
        weight_id       = weight_prior_id(i,:);
        
        % point_rot_trans before transform, which means the point_rot_trans is in model frame
        observation_ii = observation_ii + 1;
        for j = 1 : obj.num_nearestpts
            %% point_rot_trans before transform, which means the point_rot_trans is in model frame
            node_id = weight_id(j);
%             node_Rj = obj.node_rotation(3*node_id-2:3*node_id,:);
            node_gj = obj.node_position(:,node_id);
%             node_tj = obj.node_translation(:,node_id);
            node_wj = weight_prior_I(j);

            d_t = (vertex_prior_I - node_gj);
            
            Jacobian_tem = node_wj*[d_t(1)*eye(3,3),d_t(2)*eye(3,3),d_t(3)*eye(3,3),eye(3,3)];  % Jingwei's original
            
            
            Jacobian_tem2 = camera_projection_model_Jacobian(Jacobian_tem, obj.Camera{k}.R, obj.Camera{k}.t, obj.Camera{k}.s);    % jacobian of the observation term
            
            obj.Jacobian_whole(position_x+2*(observation_ii-1)+1:position_x+2*observation_ii, (node_id-1)*12+1:node_id*12) = sigma_obs * Jacobian_tem2;
            
        end
    end

end
sparse(obj.Jacobian_whole);
end