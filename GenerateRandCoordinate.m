function R = GenerateRandCoordinate(n)
% author: Jihao Liu
% data:   July 12, 2022
% function:
%         Produce a random rotation matrix where the z-axis is preset in
%         terms of the given vector, n.

% n = rand(1,3);
n = n/norm(n);
n = reshape(n,[3 1]);          % unit

temp_x = rand(3,1);
XA_t = temp_x / norm(temp_x);  % unit

YA = cross(n,XA_t);
YA = YA/norm(YA);              % unit

XA = cross(YA,n);
XA = XA/norm(XA);              % unit

R = [XA YA n];

% norm(R)
end
