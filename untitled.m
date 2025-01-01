clear;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%%       Author: Jihao Liu                                                %
%        Email: jihao.liu@sjtu.edu.cn                                     %
%        @ Shanghai Jiao Tong University                                  %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%%       0.1 Pre-set the milling planning parameters                      %
%                                                                         %
%  The number of layer and trajectory is generated by the geometrics.     %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% sensor
fr = 20;                        % sensor sample frequency, unit: Hz

%% geometrics of milling bit
R = 2.0;                        % radius, unit: mm
betaG = pi/6;                   % helix angle, unit: rad
h = 0.3;                        % segment distance, unit: mm
flag = false;                   % flag - show the edge
num_teeth = 4;                  % the number of the teeth

params.R = R;                   % radius
params.betaG = betaG;           % helix angle
params.h = h;                   % segment distance
params.flag = flag;             % flag - show the edge
params.num_teeth = num_teeth;   % the number of the teeth

%% motion parameters
rpm = 800;                      % spining speed, unit: RPM, ...
w_v = rpm * 2 * pi / 60;        % spining speed,  unit:rad/s, 800 rpm
vd  = 0.5;                      % fead rate, unit:mm/s

ft = vd/num_teeth/rpm*60;       % mm/teeth

%% milling trajectory
w = 1.5;               % distance beween adjacent trajectories on one layer, unit: mm
d = 0.8;               % the distance between layers, unit: mm

cState.w = w;          % distance beween adjacent trajectories on one layer
cState.d = d;          % the distance between layers
cState.R = R;          % the radius of the milling bit
cState.betaG = betaG;  % the helix angle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%%          0.2 Load a workspace from a CT image                          %
%           All data can load from VerSe dataset\                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load the related files.
% we will get the gray value of the block of Lumar 2.
% Limited by the copyright, please create mask files by yourself.
data_nii = load_nii('verse620_CT-iso.nii');                  % original file
data2 = load_nii('sub-verse620_dir-iso_seg-vert_msk.nii');   % mask file for L2
data3 = load_nii('verse620_lam.nii');                        % mask file for lamin.

% get the resolution of nii-file
res = data_nii.hdr.dime.pixdim(2:4);
% get the gray img of the CT image
grayVolume =  data_nii.img;

% Get the location of the pcl (of L2) segmented from the seg.msk
loc_2 = SampleNiiSeg(data2, 22, 11);     % No figure show
% Get the location of the pcl (of lami..) segmented from the lami.msk
loc_3 = SampleNiiSeg(data3, 1, 1);       % No figure show

loc_3pt = pointCloud(loc_3, 'Color',ones(length(loc_3),1)*[1 0 0]); % change the color by pointCloud

% demo - the pcl of L2
figure(1)
pcshow(loc_2);
hold on
pcshow(loc_3pt);
view([1 -1 1])     
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%%       0.3 Get a workspace filled with gray by planning                 %
%                                                                         %
%        % physical size, unit: mm                                        %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the size of the workspace
Frame = [32.168, 8.9997, 9.1966];
% the center of the workspace works as the origine.
tran  = [0.13428, -0.91245, -0.38654, 106.04;
        -0.24615, -0.40856,  0.87891,  91.693;
        -0.95988, -0.02287, -0.27946, 183.55;
               0,        0,        0,      1];

% demonstrate the frame of the workspace
frameColor = [1 1 1];
temp_pc = GenerateSquareFrame(Frame, tran, frameColor);
hold on
pcshow(temp_pc);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%%        1.  produce the cutting edge on the half sphere.                %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
trajectoryLength = Frame(1) - 2*R;
trajectoryWidth  = Frame(2) - 2*R;
millingDepth     = Frame(3);

numPointLine    = floor(trajectoryLength / vd * fr);      % the number of sampling point of every trajectory
Num_Trajecotry  = floor(trajectoryWidth / w) + 1;         % the number of milling trajectories of every layer
Num_Layer       = floor(millingDepth / d) + 1;            % the number of milling layers

% Modify the frame size. The centor / origine of the workspace does not
% move. 
new_Frame     = zeros(size(Frame));
new_Frame(1)  = numPointLine / fr * vd;          % unit: mm
new_Frame(2)  = (Num_Trajecotry - 1) * w;        % unit: mm
new_Frame(3)  = (Num_Layer - 1) * d;             % unit: mm

% %%%%
Ori_loc     = [-new_Frame/2 1] + [0, 0, d-R, 0];    % the workspace original in the LCS (coordinate system of lam.)
Ori_wld     = tran*Ori_loc';                        % the workspace original in the WCS ()
tran_wld    = tran;                                 % remodify the translation matrix.
tran_wld(1:3,4)  = Ori_wld(1:3,1);
% the tran_wld indicates the workspace coordinate system referring to the
% CT.

% produce the cutting edge
Points  = GenerateEdge2(params);               % [x, y, z, theta, phi] of edge element
points  = Points(1:3,:);                       % get the D-Position, [x, y, z] of the edge element
PCL     = [points; ones(1,length(Points))];    % tranlsate the data into the translation data
cen     = [0 0 0 1]';                          % the centroid of the milling bit

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%%       2.  milling bit moves to the initial location.                   %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the posture of milling rotation axis (MCS) in terms of the workpiece (WCS) 
RX   = ROX(0);
RY   = ROY(pi/6); % pi/10

phase_flag   = false;
%% randly given the initial phase for every milling trajectory
phase_init   = GenerateRandPhase(Num_Layer, Num_Trajecotry,phase_flag);

%%  calculate ...
delta_theta = 0.0710;             % for getting the valid edge, and area.
% d_l = R*delta_theta;

Fx = [];
Fy = [];
Fz = [];
M  = [];
gray = [];
result_m = [];          % restore the prediction

cen_loc = [];           % size is similiar to Fx, Fy, Fz, M, and gray
cen_tra = [];
edge_loc = [];          % size or dimension is higher than upper various.

Ft = [ft 0 0]';

% The tanslation matrix to the work_space ...
TM_gray = tran_wld;

% To produce the instance virtual forces and moment
for i = 1:Num_Layer
    cState.NumLayer = i;
%     i
    for j = 1:Num_Trajecotry
        cState.NumTrajectory = j;
%         j
        phase_rand = phase_init(i,j);
%         phase_init = [phase_init, phase_rand];

        temp_fx = [];
        temp_fy = [];
        temp_fz = [];
        temp_m  = [];
        temp_gray = [];
        temp_cen = [];
        temp_cen_loc = [];
        
%         count_show = 1;
        for k = 1:numPointLine
            % at a certain position.
            clock = k/fr;
            Vd = [vd * clock; w*(j-1); d*(i-1)];

            RZ = ROZ(clock*w_v + phase_rand);
            T_work = [RX*RY*RZ Vd; 0 0 0 1];
            PCL_cur = zeros(size(PCL));

            F_t = (RX*RY*RZ)\Ft;       % for t_n
            cState.F_t = F_t;          

            for m = 1:length(PCL)
                PCL_cur(:,m) = T_work * PCL(:,m);
            end
            cen_cur = T_work * cen;

            % produce the points on the 
            PCL_cur = [PCL_cur; Points(4:5,:)];
            PCLx = CuttingSurfaceX(PCL_cur,cen_cur);   % delete the half points
            valid_PCL = CWE(PCLx, cState);             % produce the parameter

            %% instance trajectory of cutting edges
            curr_cen = TM_gray * cen_cur;
            valid_wPCL = CWE2CT(valid_PCL,TM_gray);
            cen_loc(:,k,j,i) = curr_cen;
            [~,num_e] = size(valid_wPCL);
            edge_loc(:,1:num_e,k,j,i) = valid_wPCL;

            %% Force or moment evaluation
            mbForce = InstanceForceTran(valid_PCL, res, grayVolume, R, delta_theta, TM_gray);  % @ mbcs
            t_gray = GrayMeanTran(valid_PCL, res, grayVolume, TM_gray);                      %
            
            T_z = eye(4);
            T_z(1:3,1:3) = RZ;

            temp_force = T_z * mbForce;  % THIS IS SPECIAL CALCULATION NOT HOM.    % the force translated into TCS from the rotating MTCS

            temp_fx = [temp_fx temp_force(1)];
            temp_fy = [temp_fy temp_force(2)];
            temp_fz = [temp_fz temp_force(3)];
            temp_m  = [temp_m  temp_force(4)];
            temp_gray = [temp_gray t_gray];
            temp_cen = [temp_cen cen_cur];
            temp_cen_loc = [temp_cen_loc curr_cen];
        end   % data of every trajectory is produced

        % every row indicates predictions of one trajectory in the following various.
        Fx = [Fx; temp_fx];
        Fy = [Fy; temp_fy];
        Fz = [Fz; temp_fz];
        M  = [M;  temp_m];
        gray = [gray; temp_gray];

        cen_loc = [cen_loc temp_cen_loc];           % size is similiar to Fx, Fy, Fz, M, and gray
        cen_tra = [cen_tra temp_cen];


        result_m(1,:,j,i) = temp_fx;
        result_m(2,:,j,i) = temp_fy;
        result_m(3,:,j,i) = temp_fz;
        result_m(4,:,j,i) = temp_m;
        result_m(5,:,j,i) = temp_gray;

        figure(1)
        hold on
        l
        hold on

        
    end
end

%% Simulation _ The tanslation matrix to the work_space ...
tran_loc  = RandInitPoint();
Sim_Tran = tran_loc*tran_wld;
% RandInitPoint := 2 mm - randn;
% x-angle := 5 degree * randn;
% z-angle := 2 degree * randn;

sim_Fx = [];
sim_Fy = [];
sim_Fz = [];
sim_M  = [];
sim_gray = [];

sim_result = [];

sim_cen_loc = [];           % size is similiar to Fx, Fy, Fz, M, and gray
sim_edge_loc = [];          % size or dimension is higher than upper various.

%% given the initial phase for every milling trajectory
% phase_init   = GenerateRandPhase(Num_Layer, Num_Trajecotry,phase_flag);

for i = 1:Num_Layer
    cState.NumLayer = i;
%     i
    for j = 1:Num_Trajecotry
        cState.NumTrajectory = j;
%         j
        phase_rand = phase_init(i,j);
%         phase_init = [phase_init, phase_rand];

        temp_fx = [];
        temp_fy = [];
        temp_fz = [];
        temp_m  = [];
        temp_gray = [];
        
         for k = 1:numPointLine
            % at a certain position.
            clock = k/fr;
            Vd = [vd * clock; w*(j-1); d*(i-1)];

            RZ = ROZ(clock*w_v + phase_rand);
            T_work = [RX*RY*RZ Vd; 0 0 0 1];
            PCL_cur = zeros(size(PCL));

            F_t = (RX*RY*RZ)\Ft;       % for t_n
            cState.F_t = F_t;          

            for m = 1:length(PCL)
                PCL_cur(:,m) = T_work * PCL(:,m);
            end
            cen_cur = T_work * cen;

            % produce the points on the 
            PCL_cur = [PCL_cur; Points(4:5,:)];
            PCLx = CuttingSurfaceX(PCL_cur,cen_cur);   % delete the half points
            valid_PCL = CWE(PCLx, cState);             % produce the parameter

            %%
            curr_cen = Sim_Tran * cen_cur;
            valid_wPCL = CWE2CT(valid_PCL, Sim_Tran);

            sim_cen_loc(:,k,j,i) = curr_cen;
            [~,num_e] = size(valid_wPCL);
            sim_edge_loc(:,1:num_e,k,j,i) = valid_wPCL;

            %%
            mbForce = InstanceForceTran(valid_PCL, res, grayVolume, R, delta_theta, Sim_Tran);  % @ mbcs
            t_gray = GrayMeanTran(valid_PCL, res, grayVolume, Sim_Tran);                      %
            
            T_z = eye(4);
            T_z(1:3,1:3) = RZ;

            temp_force = T_z * mbForce;  % THIS IS SPECIAL CALCULATION NOT HOM.    % the force translated into TCS from the rotating MTCS

            temp_fx = [temp_fx temp_force(1)];
            temp_fy = [temp_fy temp_force(2)];
            temp_fz = [temp_fz temp_force(3)];
            temp_m  = [temp_m  temp_force(4)];
            temp_gray = [temp_gray t_gray];
        end   % data of every trajectory is produced

        % every row indicates predictions of one trajectory in the following various.
        sim_Fx = [sim_Fx; temp_fx];
        sim_Fy = [sim_Fy; temp_fy];
        sim_Fz = [sim_Fz; temp_fz];
        sim_M  = [sim_M;  temp_m];
        sim_gray = [sim_gray; temp_gray];

        sim_result(1,:,j,i) = temp_fx;
        sim_result(2,:,j,i) = temp_fy;
        sim_result(3,:,j,i) = temp_fz;
        sim_result(4,:,j,i) = temp_m;
        sim_result(5,:,j,i) = temp_gray;
        
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

save result_new0801.mat result_m sim_result



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[tra_num, sample_num] = size(Fx);
Time_scale = [1:sample_num]/fr;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%% gray analysis
%
gray_mean = mean(gray,2);
gray_max = max(gray,[],2);
gray_min = min(gray,[],2);

sim_graygray_mean = mean(sim_gray,2);
sim_graygray_max = max(sim_gray,[],2);
sim_graygray_min = min(sim_gray,[],2);


figure()
plot(gray_mean,'r');
hold on
plot(gray_mean,'g');

plot(gray_max, 'ro')
plot(gray_min, 'ro')

plot(gray_max, 'go')
plot(gray_min, 'go')
hold off
legend('Plan - GrayMean','Sim - GrayMean', 'Plan - MaxGray', 'Plan - MinGray',...
    'Sim - MaxGray', 'Sim - MinGray');

% figure()
% index = 1;
% for i = 1:Num_Layer
%     for j = 1:Num_Trajecotry
%         subplot(Num_Layer, Num_Trajecotry,index)
%         plot(Fx(index,:));
%         hold on
%         grid on
%         plot(Sim_Fx(index,:));
%         legned('Planning', 'Simulation')
%         title('Fx')
%         index = index + 1;
%     end
% end
% 
% figure()
% index = 1;
% for i = 1:Num_Layer
%     for j = 1:Num_Trajecotry
%         subplot(Num_Layer, Num_Trajecotry,index)
%         plot(Fy(index,:));
%         hold on
%         grid on
%         plot(Sim_Fy(index,:));
%         legned('Planning', 'Simulation')
%         title('Fy')
%         index = index + 1;
%     end
% end
% 
% figure()
% index = 1;
% for i = 1:Num_Layer
%     for j = 1:Num_Trajecotry
%         subplot(Num_Layer, Num_Trajecotry,index)
%         plot(Fz(index,:));
%         hold on
%         grid on
%         plot(Sim_Fz(index,:));
%         legned('Planning', 'Simulation')
%         title('Fz')
%         index = index + 1;
%     end
% end
% 
% figure()
% index = 1;
% for i = 1:Num_Layer
%     for j = 1:Num_Trajecotry
%         subplot(Num_Layer, Num_Trajecotry,index)
%         plot(M(index,:));
%         hold on
%         grid on
%         plot(Sim_M(index,:));
%         legned('Planning', 'Simulation')
%         title('Mz')
%         index = index + 1;
%     end
% end
% 
% figure()
% index = 1;
% for i = 1:Num_Layer
%     for j = 1:Num_Trajecotry
%         subplot(Num_Layer, Num_Trajecotry,index)
%         plot(gray(index,:));
%         hold on
%         grid on
%         plot(Sim_gray(index,:));
%         legned('Planning', 'Simulation')
%         title('gray')
%         index = index + 1;
%     end
% end



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% version: 2.0  -  July 31, 2022
%   - link_functions
%     RandInitPoint ()
%     Show - Gray
%     
%     ShowGrayStatistics()
%     save result.mat

