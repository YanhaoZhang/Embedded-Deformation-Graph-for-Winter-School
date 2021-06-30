function [ weights_id, weights_normalized ] = updateWeight_knn(pointcloud1,pointcloud2,num_nearestpts)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: pointcloud2 are reference point cloud, for each point in
%             pointcloud1, calculate its weight to "num_nearestpts"
%             reference points in pointcloud1 based on distance
%   Method:   weight = distance(i,j) / total_distance
%   Input:    pointcloud1:     Predicted depth and nodes. N��3
%             pointcloud2:     Current depth image        M��3
%             num_nearestpts:  number of nearest points to pointcloud2
%   Returns:  weight:          N��2*num_nearestpts. Each row is each point
%                              in pointcloud1, odd column is the index to
%                              pointcloud2, even column is the weight.
%                              All weights are norminalized.
%   Author:   Jingwei Song.   12/06/2016 


%  Yanhao20190330: modify for speed use knn    paper embedded deformation Eq. (4)
%  Jingwei's previous code is too slow for using brutle search. Here I just use MATLAB function: knnsearch
%   output: weights_id       N*num_nearestpts represents the id of pointcloud2 of each nearest points from pointcloud1
%           weights          N*num_nearestpts: weights
%   % /usr/local/MATLAB/R2018a/toolbox/stats/stats/knnsearch.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% use knnsearch to calculate distance and id. core of this function
[id_raw,dist_raw] = knnsearch(pointcloud2,pointcloud1,'K',num_nearestpts+1);    % point2 point1

dist_last         = dist_raw(:,num_nearestpts+1);


num_pointcloud1  = size(pointcloud1,1);
tem_ones = ones(num_pointcloud1,num_nearestpts);
weights_raw = tem_ones - dist_raw(:,1:num_nearestpts)./ repmat(dist_last,1,num_nearestpts);  % weight_temp(j,2) = (1 - weight_temp(j,2) / weight_temp(num_nearestpts + 1,2));   %%%Yanhao20181225: Eq. (4) 



% output
weights_id = id_raw(:,1:num_nearestpts);     % k+1 nearest is for calculate weight

tem_weight_sum = sum(weights_raw, 2);
weights_normalized = weights_raw ./ repmat(tem_weight_sum,1,num_nearestpts);
end