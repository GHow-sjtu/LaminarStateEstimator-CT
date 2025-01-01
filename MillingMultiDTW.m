function SM = MillingMultiDTW(seq,predS)
% author:   Jihao Liu
% data:     July 7, 2022
% function: 
% i1 <= N, the number of the sub-sequence that the data flow is divided
% i2 <= k, the number of the sub-sequence that the trajectory is divided
% i3 <= m, the number of the trajectories of every layer
% i4 <= l, the number of the layers

[N, n] = size(seq);
[k,m,l] = size(predS);

len = floor(k/n);    % i2 < 

C = zeros(N,len,m,l);  % DTW-metric matrix

for i4 = 1:l
    for i3=1:m
        for i2=1:len
            for i1 = 1:N
                C(i1,i2,i3,i4) = dtw(seq(i1,:),predS(1+(i2-1)*n:i2*n,i3,i4));
            end
        end
    end
end
SM = C;

end