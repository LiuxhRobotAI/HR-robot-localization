function [BS]=BSPosition3D(BSN)
% Position of the anchors
BS = [  0       ,0       ,0     ;
        6       ,0       ,0     ;
        0       ,5       ,0     ;
        3.5     ,3       ,0     ;
        3       ,2.5000  ,2.5000];

if nargin==1 
if length(BS) < BSN
    fprintf("Error: required too much anchors, input BSN = %d.", BSN);
end

if BSN <= 3
    fprintf("Error: required too less anchors, input BSN = %d.", BSN);
end

if BSN < length(BS)
    BS([length(BS) BSN],:)=BS([BSN length(BS)],:);
end

end

