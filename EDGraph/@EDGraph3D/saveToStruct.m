function [s_result] = saveToStruct(obj)
%saveToStruct: save object ot struct
% https://www.mathworks.com/matlabcentral/answers/97188-is-it-possible-to-load-my-user-defined-object-into-matlab-without-the-class-definition-file


varname = inputname(1);
props = properties(obj);
for p = 1:numel(props)
    s_result.(props{p})=obj.(props{p});
end

end

