function [Theta, Trans, Scale, num_frames] = pose_load(obj, Poses)
% rewrite pose


num_frames   = size(Poses,1);
Theta = cell(num_frames,1);
Trans = cell(num_frames,1);
Scale = cell(num_frames,1);
for k=1:num_frames
    Theta{k} = Poses{k,1};
    Trans{k} = Poses{k,2};
    Scale{k} = Poses{k,3};
end

end