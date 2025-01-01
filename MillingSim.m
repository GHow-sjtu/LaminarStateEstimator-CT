clear;
close all;
clc;
% author: Jihao Liu
% date:   June 2022
% purpose: this is used to simulate milling on a generated volume.

gray_max = 1400;
gray_min = 0;

layerNum = 10;
inclineAngle = pi/3;
grayRange = [gray_min, gray_max];
res = [0.25, 0.25, 0.25];
workSpace= [12, 12, 6];

grayVolume = CreateGray4Mill(layerNum, inclineAngle, grayRange, res, workSpace);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%       1.  produce the cutting edge on the half sphere.                  %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fr = 20;  % unit: Hz

filename = 'edge_ig.gif';

% geometrics of milling bit
params.R = 2.0;
params.betaG = pi/6;
params.h = 0.3;
params.flag = false;

% motion parameters
w_v = 800 * 2 * pi / 60;  % rad/s, 800 rpm
vd  = 0.5;                % mm/s

% produce the edge
R = params.R;
Points=GenerateEdge2(params);
points = Points(1:3,:);
PCL  = [points; ones(1,length(Points))];
cen  = [0 0 0 1]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%       2.  milling bit moves to the initial location.                    %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cState.w = 2.8;
cState.d = 0.8;
cState.R = R;

w = cState.w;
d = cState.d;

% the posture adjust
RX = ROX(pi/8);
RY = ROY(pi/10);

Fx = [];
Fy = [];
M  = [];



for i = 1:3
    cState.NumLayer = i;
%     i
    for j = 1:4
        cState.NumTrajectory = j;
%         j
        phase_rand = rand(1);
        for k = 1:320
            clock = k/fr;

            Vd = clock * [vd; 0; 0] + [R; w*(j-1)+R; d*i-R];
            
            RZ = ROZ(clock*w_v+phase_rand);
            T_work = [RX*RY*RZ Vd; 0 0 0 1];

            PCL_cur = zeros(size(PCL));

            for m = 1:length(PCL)
                PCL_cur(:,m) = T_work * PCL(:,m);
            end
            cen_cur = T_work * cen;

            PCL_cur = [PCL_cur; Points(4:5,:)];
            PCLx = CuttingSurfaceX(PCL_cur,cen_cur);
            valid_PCL = CWE(PCLx, cState);

            force = InstanceForce(valid_PCL, res, grayVolume);

            temp_force = RZ * force;

            Fx = [Fx temp_force(1)];
            Fy = [Fy temp_force(2)];
            M  = [M temp_force(3)];
        end
        Fx = [Fx zeros(1,3)];
        Fy = [Fy zeros(1,3)];
        M  = [M zeros(1,3)];
    end
end

figure()
subplot(3,1,1)
plot(Fx)
ylabel('Fx')

subplot(3,1,2)
plot(Fy)
ylabel('Fy')

subplot(3,1,3)
plot(M)
ylabel('M')