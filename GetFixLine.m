function [Ax, cen]= GetFixLine(pc)
% author:   Jihao
% data:     July 11, 2022
% function: 

loc_x = pc(:,1);
loc_y = pc(:,2);

figure()
scatter(loc_x, loc_y, 'filled');
hold on;

p = polyfit(loc_x, loc_y, 1);
tempx = [1/p(1) 1 0];
Ax = tempx/norm(tempx);
cen = [0 p(2) 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fit_y = polyval(p,loc_x);
plot(loc_x, fit_y)
hold off

end