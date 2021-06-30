function [N] = LineNormals2D(Vertices,lineStep)
% This function calculates the normals, of the line points
% using the neighbouring points of each contour point, and 
% forward an backward differences on the end points
%
% N=LineNormals2D(V,L)
%
% inputs,
%   V : List of points/vertices 2 x M
% (optional)
%   Lines : A N x 2 list of line pieces, by indices of the vertices
%         (if not set assume Lines=[1 2; 2 3 ; ... ; M-1 M])
%
% outputs,
%   N : The normals of the Vertices 2 x M
%
% Example, Hand
%  load('testdata');
%  N=LineNormals2D(Vertices,Lines);
%  figure,
%  plot([Vertices(:,1) Vertices(:,1)+10*N(:,1)]',[Vertices(:,2) Vertices(:,2)+10*N(:,2)]');
%
% Function is written by D.Kroon University of Twente (August 2011)




% If no line-indices, assume a x(1) connected with x(2), x(3) with x(4) ...
if(nargin<2)
    Lines=[(1:(size(Vertices,1)-1))' (2:size(Vertices,1))'];
else
    Lines = [(1:(size(Vertices,1)-lineStep))' (lineStep+1:size(Vertices,1))'];
end

% Calculate tangent vectors
DT=Vertices(Lines(:,1),:)-Vertices(Lines(:,2),:);        

% % calculate tangent vector of each line_step 
% DT_all = cell(1,line_step); 
% for i=1:line_step
%     Lines = Lines_all{i};   % get the id from line step
%     
%     % tangent using current line_step
%     DT = vertices(Lines(:,2),:)-vertices(Lines(:,1),:);    %[5:end]  [1:end-4]
%     
%     % normalize the tangent
%     LL=sqrt(DT(:,1).^2 + DT(:,2).^2);
%     DT(:,1)=DT(:,1)./max(LL,eps);
%     DT(:,2)=DT(:,2)./max(LL,eps);
% 
%     % store current tangent
%     DT_all{i} = DT;   
% end
% 
% % combine all tangent vector This is the tangent used
% DT = zeros(num_v,2);
% for i=1:line_step
%     DT = DT + DT_all{i};
% end


% Make influence of tangent vector 1/Distance
% (Weighted Central Differences. Points which are closer give a 
% more accurate estimate of the normal)
LL=sqrt(DT(:,1).^2+DT(:,2).^2);
DT(:,1)=DT(:,1)./max(LL.^2,eps);     % should be LL, LL.^2 is for weight the tangent
DT(:,2)=DT(:,2)./max(LL.^2,eps);

D1=zeros(size(Vertices)); D1(Lines(:,1),:)=DT;
D2=zeros(size(Vertices)); D2(Lines(:,2),:)=DT;
D=D1+D2;

% Normalize the normal
LL=sqrt(D(:,1).^2 + D(:,2).^2);
N(:,1)=-D(:,2)./LL;
N(:,2)= D(:,1)./LL;

% tmp = normc(D);

end
