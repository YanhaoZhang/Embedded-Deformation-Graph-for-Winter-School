classdef AortaDef2D < handle
    %% object of AortaDef2D. 
    % This version calculate correspondence inside GN iteration
    
    % properties for deformation default value
    properties
        num_nearestpts = 6;        %   for calculating weights
        grid_size      = 20;       %   Downsampling grid size 
    end
    
    % properties for pixel-vertex correspondence default value
    properties
        theta_threshold   = pi/6;
        dist_threshold    = 30;
        NormalVectorLines = 8;       % how many adjacent points used for calculating normal vector
        alphaShape = 3.5;            % threshold ofr alpha shape.
    end
    
    % model vertices
    properties
        num_modelVertices;          % number of model vertex
        modelVertices;              % 3*N, model vertex.
        
        vertices_weights;           % vertex weight of each node
        vertices_weights_id;
    end
    
    % core properity for ED graph (state variables are Rotation and translation)
    properties
        num_nodes;
        num_connection;    % this is number of neighbouring nodes of each node. Default value is from song's acra. This term is for E_reg
        num_weight;        % for calculate control nodes
        
        node_position;
        node_weight;       % It is actually not used
        node_weight_id;    % The id is useful
        
        node_rotation;     % deformation parameters
        node_translation;
    end
    
    % observation properity
    properties
        num_frames=2;            % number of frames used 
        Camera;
        
        Observation;

        % corresponding vertices before and after. cell of 1*num_frames
        control_vertex_prior;           % 3D position of observed vertices beform deformation, they are in model frame
        control_vertex_prior_weight;    % node weight of control_prior
        control_vertex_prior_weight_id; % node weight id of control_prior
        control_vertex_after;           % observation used as control after 2D pixel coordinate
        
        num_controlVertices;     % total number of control vertices (for jacobian and error)
        num_control_pts_all   % all transformed points (for jacobian and error)
    end
    
    % used for optimization
    properties
        % weight for the three terms
        sigma_rot = sqrt(1/1);         
        sigma_reg = sqrt(1/0.1);
        sigma_obs = sqrt(1/0.01);

        iter = 8;
        StopIterationCriteria = 0.1;
        
        Jacobian_whole;              % jacobian matrix
        Error_whole;                 % error matrix

    end
    
    
    
    methods
        %% constructor
        function [obj] = AortaDef2D(model, Camera, Observation)            
            % model inputs
            obj.modelVertices     = model.vertices';           % initialize modelVertices 3*N 和之前版本不一样
            obj.num_modelVertices = size(obj.modelVertices,2);

            % load camera
            obj.Camera = Camera;
            
            % load observation
            obj.Observation = Observation;
            
            % initialize ED nodes
            node_initialization(obj);
            
            % tuning connection number for stronger smoothness
            obj.num_connection = floor(obj.num_weight/2)+1;     %for calculating weights of ED_nodes

            % initialize control points
            control_pts_initialization(obj);
            
            % perform deformation of different iterations
            Gauss_Newton_Optimization(obj);
            
            
        end
    end
    
    %% define other function methods
    methods
        node_initialization(obj);         % initialize ED nodes
        
        control_pts_initialization(obj);  % initialize control point and node correspondence

        Gauss_Newton_Optimization(obj);   % gaussian newton iteration
        calculate_Jacobian(obj);
        calculate_Error(obj);
        [x] = x_initialization(obj);             % used for optimization
        
        update_node_parameter(obj,x);     % only update rotation and translation, not position of nodes
        [points_new] = update_pts_position(obj, points, num_nearestpts);      % update points accouding to new EDNode rotation and translation
        update_reinitialize_graph(obj);       % re-initialize deformation parameters
        
        [s_result] = saveToStruct(obj);   % output
    end
    
end