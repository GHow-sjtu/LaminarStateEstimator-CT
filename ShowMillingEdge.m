function ShowMillingEdge(Points, TM, color)
% author: Jihao Liu
% date  : July 14, 2022

[~, num] = size(Points);
act_points = zeros(num,3);

% TM : phisical space.
R_t = TM(1:3,1:3);
T_t = TM(1:3,4);

for i = 1:num
    act_points(i,:) = (R_t*Points(1:3,i) + T_t)';
end

pc_edge = pointCloud(act_points, "Color", ones(num,1)*color);

hold on
pcshow(pc_edge);

end