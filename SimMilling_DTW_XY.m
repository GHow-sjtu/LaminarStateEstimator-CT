clear;
close all;
clc;

% author: Jihao Liu
% purpose: this is used to simulate milling on a volume whose gray value is
% generated

gray_max = 1400;
gray_min = 0;

layerNum = 10;
inclineAngle = pi/3;
grayRange = [gray_min, gray_max];
res = [0.25, 0.25, 0.25];
workSpace= [12, 12, 6];

grayVolume = CreateGray4Mill(layerNum, inclineAngle, grayRange, res, workSpace);

grayVolume = TXYGray(grayVolume);

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

phase_init = [];

for i = 1:3
    cState.NumLayer = i;
%     i
    for j = 1:4
        cState.NumTrajectory = j;
%         j
        phase_rand = rand(1);
        phase_init = [phase_init, phase_rand];
        temp_fx = [];
        temp_fy = [];
        temp_m = [];
        
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

            temp_fx = [temp_fx temp_force(1)];
            temp_fy = [temp_fy temp_force(2)];
            temp_m  = [temp_m temp_force(3)];
        end
        Fx = [Fx; temp_fx];
        Fy = [Fy; temp_fy];
        M  = [M;  temp_m];
    end
end

Time_scale = [1:length(Fx)]/fr;

plot_FX = [];
plot_FY = [];
plot_M  = [];
for i = 1:12
    plot_FX = [plot_FX Fx(i,:)];
    plot_FY = [plot_FY Fy(i,:)];
    plot_M = [plot_M M(i,:)];
end

figure()
subplot(3,1,1)
plot(plot_FX)
subplot(3,1,2)
plot(plot_FY)
subplot(3,1,3)
plot(plot_M)
% 
% figure()
% % subplot(3,1,1)
% plot(Time_scale, Fx')
% ylabel('Fx')
% legend('Top_1','Top_2','Top_3', 'Top_4','Sec_1','Sec_2','Sec_3', 'Sec_4','Bot_1','Bot_2','Bot_3', 'Bot_4')
% 
% figure()
% % subplot(3,1,2)
% plot(Time_scale, Fy')
% ylabel('Fy')
% legend('Top_1','Top_2','Top_3', 'Top_4','Sec_1','Sec_2','Sec_3', 'Sec_4','Bot_1','Bot_2','Bot_3', 'Bot_4')
% 
% figure()
% % subplot(3,1,3)
% plot(Time_scale, M')
% ylabel('M')
% legend('Top_1','Top_2','Top_3', 'Top_4','Sec_1','Sec_2','Sec_3', 'Sec_4','Bot_1','Bot_2','Bot_3', 'Bot_4')
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
step = 4;
Filter_Fx = [];
Filter_Fy = [];
Filter_M  = [];

for i = 1:12
    temp_ffx = ForceMeanFilter(Fx(i,:),step);
    Filter_Fx = [Filter_Fx; temp_ffx];

    temp_ffy = ForceMeanFilter(Fy(i,:),step);
    Filter_Fy = [Filter_Fy; temp_ffy];

    temp_fm = ForceMeanFilter(M(i,:),step);
    Filter_M = [Filter_M; temp_fm];

%     Filter_Scale = [1:length(temp_filter)]/fr*step;
%     figure()
%     plot(Time_scale, Fx(i,:), Filter_Scale, temp_ffx)
%     grid on
%     figure()
%     plot(Time_scale, Fx(i,:), Filter_Scale, temp_ffy)
%     grid on
%     figure()
%     plot(Time_scale, Fx(i,:), Filter_Scale, temp_fm)
%     grid on
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% speical points
% 1,  2,  3,  4,
% 5,  6,  7,  8,
% 9, 10, 11, 12.
id = [1,4,9,10,12];

% for i=1:length(id)
%     for j = 1:length(id)
%         if (i ~= j) & (i < j)
%             figure()
%             dtw(Fx(id(i),:),Fx(id(j),:))
%             P = {num2str(id(i)), num2str(id(j))};
%             legend(P);
%         end
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%

SX = zeros(length(id));
SY = zeros(length(id));
SZ = zeros(length(id));

SfX = zeros(length(id));
SfY = zeros(length(id));
SfZ = zeros(length(id));


for i = 1:length(id)
    for j = 1:length(id)
        CoX = corrcoef(Fx(id(i),:), Fx(id(j),:));
        SX(i,j) = abs(CoX(1,2));

        CoY = corrcoef(Fy(id(i),:), Fy(id(j),:));
        SY(i,j) = abs(CoY(1,2));

        CoZ = corrcoef(M(id(i),:), M(id(j),:));
        SZ(i,j) = abs(CoZ(1,2));

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        CofX = corrcoef(Filter_Fx(id(i),:), Filter_Fx(id(j),:));
        SfX(i,j) = abs(CofX(1,2));

        CofY = corrcoef(Filter_Fy(id(i),:), Filter_Fy(id(j),:));
        SfY(i,j) = abs(CofY(1,2));

        CofZ = corrcoef(Filter_M(id(i),:), Filter_M(id(j),:));
        SfZ(i,j) = abs(CofZ(1,2));

    end
end

figure
subplot(2,3,1)
imagesc(SX)
title('Corrcoef of FX')
colorbar
subplot(2,3,2)
imagesc(SY)
title('Corrcoef of FY')
colorbar
subplot(2,3,3)
imagesc(SZ)
title('Corrcoef of MZ')
colorbar
subplot(2,3,4)
imagesc(SfX)
title('Corrcoef of filter_FX')
colorbar
subplot(2,3,5)
imagesc(SfY)
title('Corrcoef of filter_FY')
colorbar
subplot(2,3,6)
imagesc(SfZ)
title('Corrcoef of filter_MZ')
colorbar


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% top layer
%
% 1,  2,  3,  4,
% 5,  6,  7,  8,
% 9, 10, 11, 12.

% 2, 3, 4.
id = [2,3,4];

% for i=1:length(id)
%     for j = 1:length(id)
%         if (i ~= j) & (i < j)
%             figure()
%             dtw(Fx(id(i),:),Fx(id(j),:))
%             P = {num2str(id(i)), num2str(id(j))};
%             legend(P);
%         end
%     end
% 
% end

% pause(4)

%%%%%%%%%%%%%%%%%%%%%%%%%

SX = zeros(length(id));
SY = zeros(length(id));
SZ = zeros(length(id));

SfX = zeros(length(id));
SfY = zeros(length(id));
SfZ = zeros(length(id));


for i = 1:length(id)
    for j = 1:length(id)
        CoX = corrcoef(Fx(id(i),:), Fx(id(j),:));
        SX(i,j) = abs(CoX(1,2));

        CoY = corrcoef(Fy(id(i),:), Fy(id(j),:));
        SY(i,j) = abs(CoY(1,2));

        CoZ = corrcoef(M(id(i),:), M(id(j),:));
        SZ(i,j) = abs(CoZ(1,2));

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        CofX = corrcoef(Filter_Fx(id(i),:), Filter_Fx(id(j),:));
        SfX(i,j) = abs(CofX(1,2));

        CofY = corrcoef(Filter_Fy(id(i),:), Filter_Fy(id(j),:));
        SfY(i,j) = abs(CofY(1,2));

        CofZ = corrcoef(Filter_M(id(i),:), Filter_M(id(j),:));
        SfZ(i,j) = abs(CofZ(1,2));

    end
end

figure
subplot(2,3,1)
imagesc(SX)
title('Corrcoef of FX')
colorbar
subplot(2,3,2)
imagesc(SY)
title('Corrcoef of FY')
colorbar
subplot(2,3,3)
imagesc(SZ)
title('Corrcoef of MZ')
colorbar
subplot(2,3,4)
imagesc(SfX)
title('Corrcoef of filter_FX')
colorbar
subplot(2,3,5)
imagesc(SfY)
title('Corrcoef of filter_FY')
colorbar
subplot(2,3,6)
imagesc(SfZ)
title('Corrcoef of filter_MZ')
colorbar


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% middle layer
%
% 1,  2,  3,  4,
% 5,  6,  7,  8,
% 9, 10, 11, 12.

% 6,7,10,11.
id = [6,7,10,11];

% for i=1:length(id)
%     for j = 1:length(id)
%         if (i ~= j) & (i < j)
%             figure()
%             dtw(Fx(id(i),:),Fx(id(j),:))
%             P = {num2str(id(i)), num2str(id(j))};
%             legend(P);
%         end
%     end
% end
% 
% pause(4)

%%%%%%%%%%%%%%%%%%%%%%%%%

SX = zeros(length(id));
SY = zeros(length(id));
SZ = zeros(length(id));

SfX = zeros(length(id));
SfY = zeros(length(id));
SfZ = zeros(length(id));


for i = 1:length(id)
    for j = 1:length(id)
        CoX = corrcoef(Fx(id(i),:), Fx(id(j),:));
        SX(i,j) = abs(CoX(1,2));

        CoY = corrcoef(Fy(id(i),:), Fy(id(j),:));
        SY(i,j) = abs(CoY(1,2));

        CoZ = corrcoef(M(id(i),:), M(id(j),:));
        SZ(i,j) = abs(CoZ(1,2));

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        CofX = corrcoef(Filter_Fx(id(i),:), Filter_Fx(id(j),:));
        SfX(i,j) = abs(CofX(1,2));

        CofY = corrcoef(Filter_Fy(id(i),:), Filter_Fy(id(j),:));
        SfY(i,j) = abs(CofY(1,2));

        CofZ = corrcoef(Filter_M(id(i),:), Filter_M(id(j),:));
        SfZ(i,j) = abs(CofZ(1,2));

    end
end

phase_init(id)*180/pi

figure

subplot(2,3,1)
imagesc(SX)
title('Corrcoef of FX')
colorbar
subplot(2,3,2)
imagesc(SY)
title('Corrcoef of FY')
colorbar
subplot(2,3,3)
imagesc(SZ)
title('Corrcoef of MZ')
colorbar
subplot(2,3,4)
imagesc(SfX)
title('Corrcoef of filter_FX')
colorbar
subplot(2,3,5)
imagesc(SfY)
title('Corrcoef of filter_FY')
colorbar
subplot(2,3,6)
imagesc(SfZ)
title('Corrcoef of filter_MZ')
colorbar

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Simil_x = zeros(12);
F_Simil_x = zeros(12);
Simil_y = zeros(12);
F_Simil_y = zeros(12);
Simil_z = zeros(12);
F_Simil_z = zeros(12);


norm_fx = zeros(size(Fx));
norm_fy = zeros(size(Fy));
norm_m  = zeros(size(M));

norm_ffx = zeros(size(Filter_Fx));
norm_ffy = zeros(size(Filter_Fy));
norm_fm  = zeros(size(Filter_M));


for i = 1:12
    norm_fx(i,:) = mapminmax(Fx(i,:));
    norm_fy(i,:) = mapminmax(Fy(i,:));
    norm_m(i,:)  = mapminmax(M(i,:));
    
    norm_ffx(i,:) = mapminmax(Filter_Fx(i,:));
    norm_ffy(i,:) = mapminmax(Filter_Fy(i,:));
    norm_fm(i,:)  = mapminmax(Filter_M(i,:));
end

for i = 1:12
    for j=i:12
        dist = dtw(norm_fx(i,:),norm_fx(j,:));
        Simil_x(i,j) = dist;
        Simil_x(j,i) = dist;

        dist = dtw(norm_ffx(i,:),norm_ffx(j,:));
        F_Simil_x(i,j) = dist;
        F_Simil_x(j,i) = dist;

        dist = dtw(norm_fy(i,:),norm_fy(j,:));
        Simil_y(i,j) = dist;
        Simil_y(j,i) = dist;

        dist = dtw(norm_ffy(i,:),norm_ffy(j,:));
        F_Simil_y(i,j) = dist;
        F_Simil_y(j,i) = dist;

        dist = dtw(norm_m(i,:),norm_m(j,:));
        Simil_z(i,j) = dist;
        Simil_z(j,i) = dist;

        dist = dtw(norm_fm(i,:),norm_fm(j,:));
        F_Simil_z(i,j) = dist;
        F_Simil_z(j,i) = dist;

    end
end

figure()

subplot(2,3,1)
imagesc(Simil_x)
title('Similarity matrix of FX')
colorbar
subplot(2,3,2)
imagesc(Simil_y)
title('Similarity matrix of FY')
colorbar
subplot(2,3,3)
imagesc(Simil_z)
title('Similarity matrix of MZ')
colorbar
subplot(2,3,4)
imagesc(F_Simil_x)
title('Similarity matrix of filter_FX')
colorbar
subplot(2,3,5)
imagesc(F_Simil_y)
title('Similarity matrix of filter_FY')
colorbar
subplot(2,3,6)
imagesc(F_Simil_z)
title('Similarity matrix of filter_MZ')
colorbar

save resultXY_YX.mat Fx Fy M Filter_Fx Filter_Fy Filter_M


figure()
dtw(Simil_x(5,:),Simil_x(12,:))