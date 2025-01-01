function [Frame, tran] = GetFrameSize(Loc, R_CS, cen)

% author: Jihao Liu
% data:   July 12, 2022
% function: to produce a frame for the planning space.

% R_CS: the frame coordinate system (FCS) in the world coordinate system.
initP = bsxfun(@minus, Loc, cen);

tol_num = length(initP);
xmin = realmax;
ymin = realmax;
zmin = realmax;
xmax = realmin;
ymax = realmin;
zmax = realmin;

for i=1:tol_num
    temp = R_CS\initP(i,:)';
    if xmin > temp(1)
        xmin = temp(1);
    end

    if xmax < temp(1)
        xmax = temp(1);
    end

    if ymin > temp(2)
        ymin = temp(2);
    end

    if ymax < temp(2)
        ymax = temp(2);
    end

    if zmin > temp(3)
        zmin = temp(3);
    end

    if zmax < temp(3)
        zmax = temp(3);
    end
end

% adjust the center of the frame. 
delta = R_CS*[(xmax+xmin)/2 (ymax+ymin)/2 (zmax+zmin)/2]';
% new_cen = reshape(delta,[1,3]) + cen;

Frame = [xmax-xmin, ymax-ymin, zmax-zmin];

temp = eye(4);
temp(1:3,1:3) = R_CS;
temp(1:3,4) = delta + reshape(cen,[3,1]);

tran = temp;
end