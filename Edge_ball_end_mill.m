clear; clc; close all
% author: Jihao Liu
% date:   June, 2022
% function: schematic view of cutting edges on a ball-end cutter
%

R = 2.0;                              % radius, unit: mm
phi0 = 0;                             % initial phase
betaG = pi/6;                         % helix angle
h = 0.5;                              % distance between edges, unit: mm
end_s = acos(h/R);
theta_t = linspace(0,end_s,21);
xT = zeros(size(theta_t));
yT = zeros(size(theta_t));
zT = zeros(size(theta_t));

figure ()
for i = 1:length(theta_t)
    theta = theta_t(i);
    delta_phi = sin(theta)*tan(betaG);
    phi_t = phi0 + delta_phi;
    
    xT(i) = R*cos(theta)*cos(phi_t);
    yT(i) = R*cos(theta)*sin(phi_t);
    zT(i) = -R*sin(theta);
    plot3(xT(i), yT(i), zT(i), '*')
    hold on
end

phi2 = pi;
for i = 1:length(theta_t)
    theta = theta_t(i);
    delta_phi = sin(theta)*tan(betaG);
    phi_t = phi2 + delta_phi;
    
    xT(i) = R*cos(theta)*cos(phi_t);
    yT(i) = R*cos(theta)*sin(phi_t);
    zT(i) = -R*sin(theta);
    plot3(xT(i), yT(i), zT(i), '*')
    hold on
end

theta_t = linspace(0,pi/2,21);
pi1 = pi/2;
for i = 1:length(theta_t)
    theta = theta_t(i);
    delta_phi = sin(theta)*tan(betaG);
    phi_t = pi1 + delta_phi;
    
    xT(i) = R*cos(theta)*cos(phi_t);
    yT(i) = R*cos(theta)*sin(phi_t);
    zT(i) = -R*sin(theta);
    plot3(xT(i), yT(i), zT(i), '*')
    hold on
end

pi3 = 3*pi/2;
for i = 1:length(theta_t)
    theta = theta_t(i);
    delta_phi = sin(theta)*tan(betaG);
    phi_t = pi3 + delta_phi;
    
    xT(i) = R*cos(theta)*cos(phi_t);
    yT(i) = R*cos(theta)*sin(phi_t);
    zT(i) = -R*sin(theta);
    plot3(xT(i), yT(i), zT(i), '*')
    hold on
end

axis equal

[xR,yR,zR] = sphere();
[m,n] = size(xR);

cn = round(n/2);
XR = xR(cn:length(xR),:);
YR = yR(cn:length(xR),:);
ZR = -zR(cn:length(xR),:);

plot3(XR*2,YR*2,ZR*2,'Color',[204/255 204/255 204/255])

xlabel('X')
ylabel('Y')
zlabel('Z')

grid off
% plot3(xT,yT,zT,'r*')

theta = linspace(0,2*pi,41);
XT = R*cos(theta);
YT = R*sin(theta);

plot(XT,YT,'linewidth',0.5,'Color',[204/255 204/255 204/255])
% view([0,0,1])
axis([-2.2,2.2,-2.2,2.2,-2.2,0.2])

