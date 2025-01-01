clear;clc;close all;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% author: Jihao Liu
% date  : June 2024
% function: result analysis

% load a result from ...
load result_new0817.mat

% save result_new.mat result_m sim_result
[~,numPointLine,traj_num,layer_num] = size(result_m);

label = {'Fx','Fy','Fz','Mz','Gray'};

% to record the mapping ratio
click_ratio_cor = zeros(layer_num,traj_num,4);
click_ratio_dtw = zeros(layer_num,traj_num,4);

%% % post - show the mapping fitting trajectory
% test_one  = result_m(:,:,1,1);

error_list_cor = [];
error_list_dtw = [];
zeros_cor = [];
zeros_dtw = [];

mapping_cor = [];
mapping_dtw = [];

color_set = hsv(5);

% include-angle of fitting line.
angle_allow  = pi/8;
% the number of points for fitting.
research_range = 1;

%% %%%%%%%%%%%%%%%%%
% ground_true
% gt_t = ones(12,1)*[1 2 3 4];
% gt_l = [1:12]'*ones(1,4);

gt_t = zeros(layer_num,traj_num);
gt_l = zeros(layer_num,traj_num);

w = 1.5;
d = 0.8;

for count_l = 1:layer_num
    for count_t = 1:traj_num
        start_l  = sim_cen_loc(:,1,count_t,count_l);
        end_l  = sim_cen_loc(:,numPointLine,count_t,count_l);
        
        gt_t(count_l,count_t) = (start_l(2) + end_l(2))/2/w+1;
        gt_l(count_l,count_t) = (start_l(3) + end_l(3))/2/d+1;
    end
end

%%

% save - mapping result - corrcoef 
cal_cor_t = NaN([size(gt_t) 4]);
cal_cor_l = NaN([size(gt_l) 4]);
% save - mapping result - dtw
cal_dtw_t = NaN([size(gt_t) 4]);
cal_dtw_l = NaN([size(gt_l) 4]);

dist_mapping_cor = NaN(layer_num,traj_num,4);
dist_mapping_dtw = NaN(layer_num,traj_num,4);
 
MappingNum_cor = zeros(layer_num,traj_num);
MappingNum_dtw = zeros(layer_num,traj_num);

for count_l = 1:layer_num
    for count_t = 1:traj_num
        test_one = sim_result(:,:,count_t, count_l);               % target in sequence
        result_one = FindMappingPath(test_one, result_m, 30);    % planning - mapping object

        result_cor = result_one.resC;   % the location of mapping points based on CORRCOEF
        result_dtw = result_one.resD;   % the location of mapping points based on DTW
        value_cor = result_one.resCR;   % the value @ loc_cor
        value_dtw = result_one.resDR;   % the value @ loc_dtw

        [~,~,seq_num,~] = size(result_cor);

%         figure()
        for count_i = 1:4
            fittingPoints_cor = zeros(seq_num,3);

            for count_s = 1:seq_num
                fittingPoints_cor((count_s-1)*research_range+1:count_s*research_range,:) ...
                    = result_cor(1:research_range,:,count_s,count_i);
            end
%             plot3(fittingPoints_cor(:,1),fittingPoints_cor(:,2),fittingPoints_cor(:,3),'.','color',color_set(count_i,:))
%             hold on
            
            [line_cor, IDS_cor] = RANSAC3D(fittingPoints_cor);
            InPoints = fittingPoints_cor(IDS_cor,:);
            
            start_cor = line_cor(2,:);
            uvec_cor  = line_cor(1,:);
            % start_cor = zeros(x)  + vecTor * length.
            % where, length = (start_cor - zeros(x))/vectTor

%             plot3(InPoints(:,1),InPoints(:,2),InPoints(:,3),'*','color',color_set(count_i,:))
            start_zeros = start_cor - start_cor(1)*uvec_cor;            % points| x = 0;
            end_ten     = start_cor - (start_cor(1) - seq_num) * uvec_cor;   % Points| x = 80 mm;

            temp_t = (start_zeros(2) + end_ten(2))/2;
            temp_l = (start_zeros(3) + end_ten(3))/2;
            cal_cor_t(count_l,count_t,count_i) = temp_t;
            cal_cor_l(count_l,count_t,count_i) = temp_l;
            
            dist = sqrt((temp_t - gt_t(count_l,count_t))^2 ...
                + (temp_l - gt_l(count_l,count_t))^2);
                           
            dist_mapping_cor(count_l,count_t,count_i) = dist;

            click_ratio_cor(count_l,count_t,count_i) = sum(IDS_cor)/length(IDS_cor);
        end
%         str_traj = strcat("Traj: ", int2str(count_t));
%         str_layer = strcat(", Layer: ", int2str(count_l));
%         title_name = strcat(str_traj,str_layer);
%         title(strcat(title_name,"-CorrCoef"));
%         
%         figure()

        for count_i = 1:4
            fittingPoints_dtw = zeros(seq_num,3);

            for count_s = 1:seq_num
                fittingPoints_dtw((count_s-1)*research_range+1:count_s*research_range,:) ...
                    = result_dtw(1:research_range,:,count_s,count_i);
            end

%             plot3(fittingPoints_dtw(:,1),fittingPoints_dtw(:,2),fittingPoints_dtw(:,3),'.','color',color_set(count_i,:));
%             hold on
%             mapping_dtw(:,:,count_t,count_l,count_i) = mappingSeqVal_dtw;

            [line_dtw, IDS_dtw] = RANSAC3D(fittingPoints_dtw);
            InPoints = fittingPoints_dtw(IDS_dtw,:);
            
%             MappingNum_dtw(count_l,count_t) = sum(IDS_dtw);

            start_dtw = line_dtw(2,:);
            uvec_dtw  = line_dtw(1,:);
            % start_dtw = zeros(x)  + vecTor * length.
            % where, length = (start_cor - zeros(x))/vectTor

%             plot3(InPoints(:,1),InPoints(:,2),InPoints(:,3),'*','color',color_set(count_i,:))
            start_zeros = start_dtw - start_dtw(1)*uvec_dtw;            % points| x = 0;
            end_ten     = start_dtw - (start_dtw(1) - seq_num) * uvec_dtw;   % Points| x = 10 mm;
            
            temp_t = (start_zeros(2) + end_ten(2))/2;
            temp_l = (start_zeros(3) + end_ten(3))/2;
            cal_dtw_t(count_l,count_t,count_i) = temp_t;
            cal_dtw_l(count_l,count_t,count_i) = temp_l;

            dist = sqrt((temp_t - gt_t(count_l,count_t))^2 ...
                + (temp_l - gt_l(count_l,count_t))^2);
            dist_mapping_dtw(count_l,count_t,count_i) = dist;

            click_ratio_dtw(count_l,count_t,count_i) = sum(IDS_dtw)/length(IDS_dtw);
        end
%         title(strcat(title_name,"-DTW"));
    end
end

% % %% show result
% click_ratio_cor
% click_ratio_dtw

%% shown the gray-picture
figure()
for i = 1:layer_num
    for j = 1:traj_num
        subplot(layer_num, traj_num,(i-1)*traj_num + j)
        plot(result_m(5,:,j,i))
        hold on
        plot(sim_result(5,:,j,i))
    end
end


%% shown location error
for i =1:4
    figure()
    subplot(2,3,1)
    bar3(cal_cor_t(:,:,i))
    title("CorrCoef-trajectory")

    subplot(2,3,2)
    bar3(cal_cor_l(:,:,i))
    title("CorrCoef-layer")

    subplot(2,3,3)
    bar3(dist_mapping_cor(:,:,i))
    title("CorrCoef-Mapping-error")

    subplot(2,3,4)
    bar3(cal_dtw_t(:,:,i))
    title("DTW-trajectory")

    subplot(2,3,5)
    bar3(cal_dtw_l(:,:,i))
    title("DTW-layer")

    subplot(2,3,6)
    bar3(dist_mapping_dtw(:,:,i))
    title("DTW-Mapping-error")
end

%%
for i = 1:4
    figure()
    subplot(2,1,1)
    histogram(dist_mapping_cor(:,:,i))
%     hold on
%     histogram(dist_mapping_cor(1:10,1:3,i))
    title(strcat('cor-',label(i)))
    subplot(2,1,2)
    histogram(dist_mapping_dtw(:,:,i))
%     hold on
%     histogram(dist_mapping_dtw(1:10,1:3,i))
    title(strcat('dtw-',label(i)))
end

%% reg
gt_tran = zeros(1,2);
gt_tran(1) = mean(mean(gt_l - [1:layer_num]'*ones(1,traj_num)));
gt_tran(2) = mean(mean(gt_t - ones(layer_num,1)*[1:traj_num]));
% confusion
tran_cor = zeros(4,2);
tran_dtw = zeros(4,2);

for i = 1:4
    % % - close -
    tran_cor(i,:) = GetRotLoc(cal_cor_l(:,:,i), cal_cor_t(:,:,i));
    tran_dtw(i,:) = GetRotLoc(cal_dtw_l(:,:,i), cal_dtw_t(:,:,i));
end

error_cor = zeros(4,1);
error_dtw = zeros(4,1);

error_cor_t = bsxfun(@times, bsxfun(@minus, tran_cor, gt_tran), [d, w]);
error_dtw_t = bsxfun(@times, bsxfun(@minus, tran_dtw, gt_tran), [d, w]);

for i = 1:4
    error_cor(i) = norm(error_cor_t(i,:));
    error_dtw(i) = norm(error_dtw_t(i,:));
end

figure()
plot(error_cor);
hold on
plot(error_dtw);


mkdir('result_regfig')
for i=1:10
    figure(i)
    savefig(strcat(strcat('result_regfig/',int2str(i)),'.fig'))
end


%% %%%%%%%%%%%%%%%%%%

% - function
%      FindMappingPath                     -- VERSION 1.0
%      Add color to identify               -- VERSION 1.1 (deleted version in 3.0)
%      get the corrcoef / dtw-distance.    -- VERSION 2.0
%      location error analysis             -- VERSION 3.0
%      physical error evaluation           -- VERSION 3.0
%      test interaction efficiency         -- VERSION 3.0
%      add fusion - four channels          -- VERSION 4.0 (Maybe delete corr-coef)


