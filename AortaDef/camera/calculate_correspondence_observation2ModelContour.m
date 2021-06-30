function [indexCorrespondence_observation2ModelContour,observation_removedNotCorrespondendingPoint] = ...
    calculate_correspondence_observation2ModelContour(observation,observation_normal,modelContour,modelContour_normal,theta_threshold,dist_threshold)
%Calculate correspondence according to normal contour
% input: observation M*2
%        observation_normal
%        model_contour N*2 (N>M)
%        model_normal
%        theta_threshold: threshold for normal selection
%        dist_threshold: distance threshold, only the correspondence smaller than this value are output
% input data has been processed in the main scrips: calculate_downsampleObservation_normal_correspondence





% todo: calculate correspondence according to observation, observation
% normal, model_proj_contour, model_proj_nv




end