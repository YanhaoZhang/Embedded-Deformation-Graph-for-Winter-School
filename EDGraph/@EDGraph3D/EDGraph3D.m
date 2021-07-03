classdef EDGraph3D < handle
    %% object of ED graph. 
    
    % properties for deformation default value
    properties
        num_nearestpts = 6;          % for weights of ednode connection    
        grid_size      = 10/1;       % Downsampling grid size for EDnodes  
    end
    
    % 3D model 
    properties
        num_modelVertices;          % num of vertex
        modelVertices;              % position of model vertices: 3*N
    end
    
    % core properity for ED graph (affine matrix and translation)
    properties
        num_nodes;
        num_connection = 5;    % this is number of neighbouring nodes of each node. This term is for E_reg
        num_weight;             % for calculate control nodes
        
        node_position;
        node_weight_id;       % node weight
        
        node_rotation;        % deformation parameters: affine & transformation 
        node_translation;
    end
    
    % observation for control points
    properties
        control_pts_prior;              % 3D position of observed points beform deformation
        control_pts_prior_weight;       % weight of control points
        control_pts_prior_weight_id;    % weight id
        control_pts_after;              % observation of control points after deformation
        control_pts_after_normal2D;
        num_control_pts_prior;                % number of control points (for jacobian and error)
    end
    

    % used for GaussNetwon optimization
    properties
        % weight for the three terms
        sigma_rot = sqrt(1/1);         
        sigma_reg = sqrt(1/0.1);
        sigma_obs = sqrt(1/0.01);

        iter = 50;
        StopIterationCriteria = 1e-3;
        
        Jacobian_whole;              % jacobian matrix
        Error_whole;                 % error matrix
    end
    
    %% constructor
    methods
        function [obj] = EDGraph3D(Model, Obser)
            obj.modelVertices     = Model.vertices';            % 3*N
            obj.num_modelVertices = size(obj.modelVertices,2);


            % initialize ED nodes
            node_initialization(obj);
           
            % control points
            control_pts_correspondence(obj, Obser);
            
            %% perform deformation of different iterations
            Gauss_Newton_Optimization(obj);
            
        end
    end
    
    %% define function method
    methods
        node_initialization(obj);          % initialize ED graph
        control_pts_correspondence(obj, Obser);   % control point correspondence
        Gauss_Newton_Optimization(obj);    % main function for gaussian newton iteration
        calculate_Jacobian(obj);           % jacobian and
        calculate_Error(obj);
        [x] = x_initialization(obj);             % used for optimization
        
        update_node_parameter(obj,x);      % only update rotation and translation, not position of nodes
        [points_new] = update_pts_position(obj, points, num_nearestpts);      % update points accouding to new EDNode rotation and translation      
        
        
        
        [s_result] = saveToStruct(obj);   % save result
    end
    
end