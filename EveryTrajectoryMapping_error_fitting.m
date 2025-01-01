clear;clc;close all;
% author: Jihao Liu
% date:   June 2022

load result_new0803_7vvvsmall.mat


% save result_new.mat result_m sim_result
[~,numPointLine,traj_num,layer_num] = size(result_m);

label = {'Fx','Fy','Fz','Mz','Gray'};

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

% parameter for fitting
best_num = 30;

% physical size
w = 1.5;
d = 0.8;

%% %%%%%%%%%%%%%%%%%
% ground_true
gt_t = zeros(layer_num,traj_num);
gt_l = zeros(layer_num,traj_num);

% produce the ground truth. 
for count_l = 1:layer_num
    for count_t = 1:traj_num
        start_l  = sim_cen_loc(:,1,count_t,count_l);
        end_l  = sim_cen_loc(:,numPointLine,count_t,count_l);
        
        gt_t(count_l,count_t) = (start_l(2) + end_l(2))/2/w+1;
        gt_l(count_l,count_t) = (start_l(3) + end_l(3))/2/d+1;
    end
end

% calculate the error from the start-whole.
start_layer = ceil(best_num / traj_num);

% the number of the layers should be evaluated
e_layer_num = layer_num - start_layer + 1;


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

% estimate the state of every trajectory

for count_l = 1:layer_num
    for count_t = 1:traj_num
        test_one = sim_result(:,:,count_t, count_l);               % target in sequence
        result_one = FindMappingPath(test_one, result_m, 30);    % planning - mapping object

        result_cor = result_one.resC;   % the location of mapping points based on CORRCOEF
        result_dtw = result_one.resD;   % the location of mapping points based on DTW
        value_cor = result_one.resCR;   % the value @ loc_cor
        value_dtw = result_one.resDR;   % the value @ loc_dtw

        [~,~,seq_num,~] = size(result_cor);

        for count_i = 1:4
            fittingPoints_cor = zeros(seq_num,3);

            for count_s = 1:seq_num
                fittingPoints_cor((count_s-1)*research_range+1:count_s*research_range,:) ...
                    = result_cor(1:research_range,:,count_s,count_i);
            end
            
            [line_cor, IDS_cor] = RANSAC3D(fittingPoints_cor);
            InPoints = fittingPoints_cor(IDS_cor,:);
            
            start_cor = line_cor(2,:);
            uvec_cor  = line_cor(1,:);

            start_zeros = start_cor - start_cor(1)*uvec_cor;            % points| x = 0;
            end_ten     = start_cor - (start_cor(1) - seq_num) * uvec_cor;   % Points| x = 80 mm;

            temp_t = (start_zeros(2) + end_ten(2))/2;
            temp_l = (start_zeros(3) + end_ten(3))/2;
            cal_cor_t(count_l,count_t,count_i) = temp_t;
            cal_cor_l(count_l,count_t,count_i) = temp_l;
            
            dist = sqrt((temp_t - gt_t(count_l,count_t))^2 ...
                + (temp_l - gt_l(count_l,count_t))^2);
                           
            dist_mapping_cor(count_l,count_t,count_i) = dist;

        end

        for count_i = 1:4
            fittingPoints_dtw = zeros(seq_num,3);

            for count_s = 1:seq_num
                fittingPoints_dtw((count_s-1)*research_range+1:count_s*research_range,:) ...
                    = result_dtw(1:research_range,:,count_s,count_i);
            end

            [line_dtw, IDS_dtw] = RANSAC3D(fittingPoints_dtw);
            InPoints = fittingPoints_dtw(IDS_dtw,:);
            
            start_dtw = line_dtw(2,:);
            uvec_dtw  = line_dtw(1,:);

            start_zeros = start_dtw - start_dtw(1)*uvec_dtw;            % points| x = 0;
            end_ten     = start_dtw - (start_dtw(1) - seq_num) * uvec_dtw;   % Points| x = 10 mm;
            
            temp_t = (start_zeros(2) + end_ten(2))/2;
            temp_l = (start_zeros(3) + end_ten(3))/2;
            cal_dtw_t(count_l,count_t,count_i) = temp_t;
            cal_dtw_l(count_l,count_t,count_i) = temp_l;

            dist = sqrt((temp_t - gt_t(count_l,count_t))^2 ...
                + (temp_l - gt_l(count_l,count_t))^2);
            dist_mapping_dtw(count_l,count_t,count_i) = dist;

        end
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
    title(strcat('cor-',label(i)))

    subplot(2,1,2)
    histogram(dist_mapping_dtw(:,:,i))
    title(strcat('dtw-',label(i)))
end

%% fitting - evaluation
eCor_list = zeros(4, e_layer_num);
eDTW_list = zeros(4, e_layer_num);

for count_i = 1:e_layer_num
    curr_layer = start_layer + count_i - 1;    % the evulating layers

    % reference value
    gt_tran = zeros(1,2);
    gt_tran(1) = mean(mean(gt_l(1:curr_layer,:) - [1:curr_layer]'*ones(1,traj_num)));
    gt_tran(2) = mean(mean(gt_t(1:curr_layer,:) - ones(curr_layer,1)*[1:traj_num]));
    
    tran_cor = zeros(4,2);
    tran_dtw = zeros(4,2);
    
    for i = 1:4
        % % - close -
        tran_cor(i,:) = GetRotLoc(cal_cor_l(1:curr_layer,:,i), cal_cor_t(1:curr_layer,:,i));
        tran_dtw(i,:) = GetRotLoc(cal_dtw_l(1:curr_layer,:,i), cal_dtw_t(1:curr_layer,:,i));
    end
    
    error_cor = zeros(4,1);
    error_dtw = zeros(4,1);
    
    error_cor_t = bsxfun(@times, bsxfun(@minus, tran_cor, gt_tran), [d, w]);
    error_dtw_t = bsxfun(@times, bsxfun(@minus, tran_dtw, gt_tran), [d, w]);
    
    for i = 1:4
        error_cor(i) = norm(error_cor_t(i,:));
        error_dtw(i) = norm(error_dtw_t(i,:));
    end

    eCor_list(:,count_i) = error_cor;
    eDTW_list(:,count_i) = error_dtw;
end

figure()
boxplot(eCor_list,'Colors', 'g');
hold on
boxplot(eDTW_list);


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


