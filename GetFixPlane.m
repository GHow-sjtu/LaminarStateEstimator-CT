function [nv, cen] = GetFixPlane(pc)
% author:   Jihao Liu
% data:     July 11, 2022
% function: 

% loc_x = pc(:,1);
% loc_y = pc(:,2);
% loc_z = pc(:,3);
% scatter3(loc_x, loc_y, loc_z, 'filled');
hold on;
planeData = pc(:, 1:3);

% SVD
xyz0 = mean(planeData, 1);
centeredPlane = bsxfun(@minus, planeData, xyz0);
[U,S,V] = svd(centeredPlane);

a = V(1,3);
b = V(2,3);
c = V(3,3);
d = -dot([a b c], xyz0);

fprintf('%f %f %f %f', a,b,c,d)

% plot figure
xfit = min(x):0.1:max(x);
yfit = min(y):0.1:max(y);
[XFIT, YFIT] = meshgrid(xfit,yfit);
ZFIT = -(d + a*XFIT + b*YFIT)/c;
mesh(XFIT,YFIT,ZFIT);
hold off;

nv = [a b c];
cen = xyz0;
end