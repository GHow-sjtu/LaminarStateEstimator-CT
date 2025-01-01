function points = CWE2CT(PCL,TM)
% author: Jihao Liu
% date:  July 25, 2022

[~, num] = size(PCL);
temp = zeros(3,num);

R_t = TM(1:3,1:3);
T_t = TM(1:3,4);

for i = 1:num
    temp_point = R_t*PCL(1:3,i) + T_t;
    temp(:,i) = temp_point;
end

points = temp;
end

%% version: 0.1
%  function: the point in the cutting coordinate system is transformed into
%            the CT coordinate system by the various TM.