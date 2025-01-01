function points = GenerateEdgeV2(params)

% [X Y Z theta phi]'

R     = params.R;
betaG = params.betaG;
h     = params.h;

flag  = params.flag; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% R = 2.0;   % unit: mm
% betaG = pi/6;
% h = 0.5;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start_s = asin(h/R);

theta_t = linspace(start_s,pi/2,21);

xT_2 = zeros(size(theta_t));
yT_2 = zeros(size(theta_t));
zT_2 = zeros(size(theta_t));
theta_2 = zeros(size(theta_t));
phi_2 = zeros(size(theta_t));

xT_4 = zeros(size(theta_t));
yT_4 = zeros(size(theta_t));
zT_4 = zeros(size(theta_t));
theta_4 = zeros(size(theta_t));
phi_4 = zeros(size(theta_t));

phi2 = pi;
phi4 = 0;
for i = 1:length(theta_t)
    theta = theta_t(i);
    delta_phi = (1-cos(theta))*tan(betaG);

    phi_t = phi2 - delta_phi;
    xT_2(i) = R*sin(theta)*sin(phi_t);
    yT_2(i) = R*sin(theta)*cos(phi_t);
    zT_2(i) = R*cos(theta);

    theta_2(i) = theta;
    phi_2(i) = phi_t;

    phi_t = phi4 - delta_phi;
    xT_4(i) = R*sin(theta)*sin(phi_t);
    yT_4(i) = R*sin(theta)*cos(phi_t);
    zT_4(i) = R*cos(theta);
    theta_4(i) = theta;
    phi_4(i) =phi_t;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
theta_t = linspace(0,pi/2,21);

xT_1 = zeros(size(theta_t));
yT_1 = zeros(size(theta_t));
zT_1 = zeros(size(theta_t));
theta_1 = zeros(size(theta_t));
phi_1 = zeros(size(theta_t));


xT_3 = zeros(size(theta_t));
yT_3 = zeros(size(theta_t));
zT_3 = zeros(size(theta_t));
theta_3 = zeros(size(theta_t));
phi_3 = zeros(size(theta_t));

pi1 = pi/2;
pi3 = 3*pi/2;
for i = 1:length(theta_t)
    theta = theta_t(i);
    delta_phi = (1 - cos(theta))*tan(betaG);

    phi_t = pi1 - delta_phi;
    xT_1(i) = R*sin(theta)*sin(phi_t);
    yT_1(i) = R*sin(theta)*cos(phi_t);
    zT_1(i) = R*cos(theta);
    theta_1(i) = theta;
    phi_1(i) =phi_t;

    phi_t = pi3 - delta_phi;
    xT_3(i) = R*sin(theta)*sin(phi_t);
    yT_3(i) = R*sin(theta)*cos(phi_t);
    zT_3(i) = R*cos(theta);
    theta_3(i) = theta;
    phi_3(i) =phi_t;
end

xT = [xT_1 xT_2 xT_3 xT_4];
yT = [yT_1 yT_2 yT_3 yT_4];
zT = [zT_1 zT_2 zT_3 zT_4];
thetaT = [theta_1 theta_2 theta_3 theta_4];
phiT = [phi_1 phi_2 phi_3 phi_4];


if flag
    figure()
    plot3(xT,yT,zT,'*');
    hold on
    axis equal

    [xR, yR, zR] = sphere();
    [m, n] = size(xR);
    cn = round(n/2);
    XR = xR(cn:length(xR),:);
    YR = yR(cn:length(xR),:);
    ZR = zR(cn:length(xR),:);
    mesh(XR*R,YR*R,ZR*R);

    xlabel('X');
    ylabel('Y');
    zlabel('Z');

end
points = [xT; yT; zT; thetaT; phiT];
% 
% Points.x = xT;
% Points.y = yT;
% Points.z = zT;
end