clear;clc;close all;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% author: Jihao Liu
% date:   June 2022
% function: shown the engagement surface of the ball-end cutter subjct to 
%           layer-by-layer milling strategy

% milling profile and surface

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% case 1. the drill bit invading into the surface.

h = 0.5;    % unit: mm 
R = 2;      % unit: mm

w = 3.2;    % unit: mm
d = 0.5;    % unit: mm

u_d = 4;

deg = acos(h/R);

num = floor(deg * 180 /pi /u_d) + 1;

%%%
phi = linspace(0, 2*pi, 31);

X = [];
Y = [];
Z = [];

for i = 1:num
    r = R*sind((i-1)*u_d);

    x = r*cos(phi);
    y = r*sin(phi);
    z = - ones(size(phi))*R*cosd(i*u_d);

    X = [X;x];
    Y = [Y;y];
    Z = [Z;z];
end

figure()
plot3(X,Y,Z,'*')
axis equal
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
title('Valid milling surface during invading into the workpiece')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% case 2. the drill bit moving along the first line.

% clear;clc;close all;

% h = 0.8;    % unit: mm 
% R = 2;      % unit: mm

deg = acos(h/R);
% u_d = 5;

num = floor(deg * 180 /pi /u_d) + 1;

%%%
phi = linspace(0, pi, 16);

X = [];
Y = [];
Z = [];

for i = 1:num
    r = R*sind((i-1)*u_d);

    x = r*cos(phi);
    y = r*sin(phi);
    z = - ones(size(phi))*R*cosd(i*u_d);

    X = [X;x];
    Y = [Y;y];
    Z = [Z;z];
end

figure()
plot3(X,Y,Z,'*')
hold on
mesh(X,Y,Z)
surf(X,Y,Z)
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
grid on
axis equal
% title('Valid milling surface in the first line on the top layer')
set(gca,'FontName','Times New Roman','FontSize',12);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% case 3. the drill bit moving along the second line on the top layer.
% clear;clc;close all;

% h = 0.3;    % unit: mm 
% R = 2;      % unit: mm
% u_d = 4;

%%%
phi = linspace(0, pi, 26);

% w = 2.6;     % unit: mm

X = [];
Y = [];
Z = [];

cen_x = w;
cen_y = 0;
cen_z = 0;

for i = 1:num
    r = R*sind((i-1)*u_d);

    x = r*cos(phi) + cen_x;
    y = r*sin(phi) + cen_y;
    z = - ones(size(phi))*R*cosd(i*u_d) + cen_z;

    for j = 1:length(phi)
        dist = sqrt(x(j)^2 + z(j)^2);
        if dist < R
            x(j) = NaN;
            y(j) = NaN;
            z(j) = NaN;
        end
    end

    X = [X;x];
    Y = [Y;y];
    Z = [Z;z];
end

figure()
plot3(X,Y,Z,'*')
hold on
mesh(X,Y,Z)
surf(X,Y,Z)
grid on
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis equal
% title('Valid milling surface in the second line on the top layer')

set(gca,'FontName','Times New Roman','FontSize',12);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% case 4. the drill bit moving along the first line on the second layer.
% clear;clc;close all;

% h = 0.3;    % unit: mm 
% R = 2;      % unit: mm
% d = 0.8;    % unit: mm
% u_d = 4;

% num = floor(deg * 180 /pi /u_d) + 1;

phi = linspace(0, pi, 26);

% w = 2.6;     % unit: mm

X = [];
Y = [];
Z = [];

cen_x = 0;
cen_y = 0;
cen_z = -d;

for i = 1:num
    r = R*sind((i-1)*u_d);

    x = r*cos(phi) + cen_x;
    y = r*sin(phi) + cen_y;
    z = - ones(size(phi))*R*cosd(i*u_d) + cen_z;

    for j = 1:length(phi)
        dist1 = sqrt(x(j)^2 +  z(j)^2);
        dist2 = sqrt((x(j) - w)^2 + z(j)^2);


        if (dist1 < R) | (dist2 < R)
            x(j) = NaN;
            y(j) = NaN;
            z(j) = NaN;
        end
    end

    X = [X;x];
    Y = [Y;y];
    Z = [Z;z];
end

figure()
plot3(X,Y,Z,'*')
hold on
mesh(X,Y,Z)
surf(X,Y,Z)
grid on
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis equal
% title('Valid milling surface in the first line on the second layer')
set(gca,'FontName','Times New Roman','FontSize',12);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% case 5. the drill bit moving along the first line on the second layer.
% clear;clc;close all;

% h = 0.3;    % unit: mm 
% R = 2;      % unit: mm
% d = 0.8;    % unit: mm
% u_d = 4;
% num = floor(deg * 180 /pi /u_d) + 1;

%%%
% phi = linspace(0, pi, 26);
% w = 2.6;     % unit: mm

X = [];
Y = [];
Z = [];

cen_x = w;
cen_y = 0;
cen_z = -d;

for i = 1:num
    r = R*sind((i-1)*u_d);

    x = r*cos(phi) + cen_x;
    y = r*sin(phi) + cen_y;
    z = - ones(size(phi))*R*cosd(i*u_d) + cen_z;

    for j = 1:length(phi)
        dist1 = sqrt(x(j)^2 +  (z(j)+d)^2);
        dist2 = sqrt((x(j) - w)^2 + z(j)^2);
        dist3 = sqrt((x(j) - 2*w)^2 + z(j)^2);


        if (dist1 < R) | (dist2 < R) | (dist3 < R)
            x(j) = NaN;
            y(j) = NaN;
            z(j) = NaN;
        end
    end

    X = [X;x];
    Y = [Y;y];
    Z = [Z;z];
end

figure()
plot3(X,Y,Z,'*')
hold on
mesh(X,Y,Z)
surf(X,Y,Z)
grid on
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis equal
% title('Valid milling surface in the second line on the second layer')

set(gca,'FontName','Times New Roman','FontSize',12);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% case 6. the drill bit moving along the final line on the second layer.
% clear;clc;close all;

% h = 0.3;    % unit: mm 
% R = 2;      % unit: mm
% d = 0.8;    % unit: mm
% u_d = 4;

%%%
% phi = linspace(0, pi, 26);
% w = 2.6;     % unit: mm

X = [];
Y = [];
Z = [];

cen_x = w;
cen_y = 0;
cen_z = -d;

for i = 1:num
    r = R*sind((i-1)*u_d);

    x = r*cos(phi) + cen_x;
    y = r*sin(phi) + cen_y;
    z = - ones(size(phi))*R*cosd(i*u_d) + cen_z;

    for j = 1:length(phi)
        dist1 = sqrt(x(j)^2 +  (z(j)+d)^2);
        dist2 = sqrt((x(j) - w)^2 + z(j)^2);
%         dist3 = sqrt((x(j) - 2*w)^2 + z(j)^2);


        if (dist1 < R) | (dist2 < R)  %| (dist3 < R)
            x(j) = NaN;
            y(j) = NaN;
            z(j) = NaN;
        end
    end

    X = [X;x];
    Y = [Y;y];
    Z = [Z;z];
end

figure()
plot3(X,Y,Z,'*')
hold on
mesh(X,Y,Z)
surf(X,Y,Z)
grid on
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis equal
% title('Valid milling surface in the final line on the second layer')
set(gca,'FontName','Times New Roman','FontSize',12);
