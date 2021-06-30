function [ meanDist_point2point,meanDist_point2plane,errorA2B_point2point,errorA2B_point2plane ] = computeError_result2groundtruth( vertices_A, vertices_B, verticesNormal_B)
%% COMPUTEHD  -- Computes Hausdorff Distance
%                This is a traditional Hausdorff method, which is sensitive
%                to outliers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OUTPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      hd     -- Hausdorff distance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INPUTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       A     -- First point set; can be 1D, 2D, or 3D
%       B     -- Second point set; can be 1D, 2D, or 3D
% Note: Both sets A and B should have the same dimension. That is, 1D vs
% 1D; 2D vs 2D; and 3D vs 3D. The code may easily be extended to dimensions
% higher than 3D.
%  method     -- Selected metric for measuring distance:
%                abs: distance = |x2 - x1|
%              smape: distance = |x2 - x1|/(|x2| + |x1|)
%          Euclidean: distance = sqrt((x2 - x1)^2 + (y2 - y1)^2)
%                                for 2D sets
%                              = sqrt((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2)
%                                for 3D sets
%              quasi: (a) 2D sets
%                         distance = |x2 - x1| + (sqrt(2) - 1)*|y2 -y1|
%                                    if |x2 - x1| > |y2 - y1|
%                                  = (sqrt(2) - 1)*|x2 - x1| + |y2 - y1|
%                                    otherwise
%                     (b) 3D sets
%                         distance = |x2 - x1| + |y2 - y1| + (sqrt(2) -1)*|z2 - z1|
%                                    if |x2 - x1| > |z2 - z1| and |y2 - y1| > |z2 - z1|
%                                  = |x2 - x1| + (sqrt(2) -1)*|y2 - y1| + |z2 - z1|
%                                    if |x2 - x1| > |y2 - y1| and |z2 - z1| > |y2 - y1|
%                                  = (sqrt(2) -1)*|x2 - x1| + |y2 - y1| + |z2 - z1|
%                                    if |y2 - y1| > |x2 - x1| and |z2 - z1| > |x2 - x1|
%          cityblock:  (a) 2D sets
%                          distance = |x2 - x1| + |y2 - y1|
%                      (b) 3D sets
%                          distance = |x2 - x1| + |y2 - y1| + |z2 - z1|
%         chessboard:  (a) 2D sets
%                          distance = max(|x2 - x1|,|y2 - y1|)
%                      (b) 3D sets
%                          distance = max(|x2 - x1|,|y2 - y1|,|z2 - z1|)
%  A and B are input arguments are of type DOUBLE
%  hd is returned as DOUBLE
%  'method' argument is of type STRING 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Author  : Baraka Jacob Maiseli
%%% Date    : 10 March, 2017
%%% Version : 1.0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                 


%computeError_result2groundtruth: 
% vertices_A:       M*3
% vertices_B:       N*3
% verticesNormal_B: N*3


num_A = size(vertices_A,1); % Number of rows of set A
num_B = size(vertices_B,1); % Number of rows of set B



errorA2B_point2point = zeros(num_A,1);        % error of each point from set A to the nearest point from set B
ind_errorA2B_point2point = zeros(num_A,1);  % for each point from set A, the index of its nearest point from set B
errorA2B_point2plane = zeros(num_A,1);        % use normal vector to calculate point to plane error

for a=1:num_A
    %% point 2 point error
    P = repmat(vertices_A(a,:),num_B,1)-vertices_B;    % same as repmat(A(a,:),m,1)-B;
    P_dist = sqrt((P(:,1)).^2 + (P(:,2)).^2 + (P(:,3)).^2);    % only use the euclidean distance

    [dab,ind] = min(P_dist);
    errorA2B_point2point(a) = dab;
    ind_errorA2B_point2point(a) = ind;
    
    %% point 2 plane error
    v_A = vertices_A(a,:);
    v_B = vertices_B(ind,:);
    dist_A2B = v_A - v_B;
    normal_B = verticesNormal_B(ind,:);
    errorA2B_point2plane(a) = abs(dot(dist_A2B,normal_B));
end

meanDist_point2point = mean(errorA2B_point2point);

meanDist_point2plane = mean(errorA2B_point2plane);

end





