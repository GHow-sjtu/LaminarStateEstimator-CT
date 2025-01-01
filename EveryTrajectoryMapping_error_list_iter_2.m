clear;clc;close all;
% author: Jihao Liu
% date:   June 2022

%% load data_file in the format of mat
% load result_new20220726.mat
% load result_new0801.mat
% load result_new0801_2.mat
% load result_new0803.mat
% ground_true
% gt_t = ones(12,1)*[1 2 3 4];
% gt_l = [1:12]'*ones(1,4);

% load result_new0803_2small.mat
% ground_true
% gt_t = ones(12,1)*[1 2 3 4];
% gt_l = [1:12]'*ones(1,4);

% load result_new0803_3vsmall.mat  % 
% ground_true
% gt_t = ones(12,1)*[1 2 3 4];
% gt_l = [1:12]'*ones(1,4);

% load result_new0803_4vvsmall.mat
% ground_true
% gt_t = ones(12,1)*[1 2 3 4];
% gt_l = [1:12]'*ones(1,4);

% load result_new0803_6vvvsmall.mat
% ground_true
% gt_t = ones(12,1)*[1 2 3 4];
% gt_l = [1:12]'*ones(1,4);

% laod result_new0803_7vvvsmall.mat
% ground_true
% gt_t = ones(12,1)*[1 2 3 4];
% gt_l = [1:12]'*ones(1,4);

load result_new0803_7_2vvvsmall.mat

% get the parameters in the simulation
[~,numPointLine,traj_num,layer_num] = size(result_m);


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

% label = {'Fx','Fy','Fz','Mz','Gray'};

% to record the mapping ratio
click_ratio_cor = zeros(layer_num,traj_num,5,5);
click_ratio_dtw = zeros(layer_num,traj_num,5,5);

%% % post - show the mapping fitting trajectory

% include-angle of fitting line.
angle_allow  = pi/8;

%% iteration analysis
% mapping result - 
cal_cor_t = NaN([size(gt_t) 5 5]);   % trajectory_num  [layer_num, traj_num, comp_num, it_num]; 
cal_cor_l = NaN([size(gt_l) 5 5]);   % layer_num       [layer_num, traj_num, comp_num, it_num]; 

cal_dtw_t = NaN([size(gt_t) 5 5]);   % trajectory_num  [layer_num, traj_num, comp_num, it_num]; 
cal_dtw_l = NaN([size(gt_l) 5 5]);   % layer_num       [layer_num, traj_num, comp_num, it_num]; 

dist_mapping_cor = NaN(layer_num,traj_num,5,5);        % evaluation error
dist_mapping_dtw = NaN(layer_num,traj_num,5,5);        % evaluation error

% distance / coef of points from findmapping
mapping_cor = [];      % [N,3,layer_num,traj_num,5,5]
mapping_dtw = [];      % [N,3,layer_num,traj_num,5,5]

% distance / coef of fitting point in findmapping 
fittingmapping_cor = [];   % = fitting inside the line
fittingmapping_dtw = [];   % = fitting inside the line


%% calculation main part
for count_l = 1:layer_num
    for count_t = 1:traj_num
        test_one = sim_result(:,:,count_t, count_l);
        result_one = FindMappingPath(test_one, result_m, 45);

        % mapping result -- first five element according to the mappingPath.
        result_cor = result_one.resC;   % the location of mapping points based on CORRCOEF
        result_dtw = result_one.resD;   % the location of mapping points based on DTW
        value_cor = result_one.resCR;   % the value vs. loc
        value_dtw = result_one.resDR;   % the value vs. loc

        [~,~,seq_num,~] = size(result_cor);  % Get the size

        %%% main -- part --
        for count_i = 1:5
            % for interaction
            for count_it = 1:5
                % the number of points for fitting.
                research_start = 1;          % Model of single sample; if :=1, multiple sample points;
                research_range = count_it;          % Model of single sample; if :=1, multiple sample points;

                %% core part
                %%%%%%%%%%%%%%%%% corr-coef
                fittingPoints_cor = zeros(seq_num*(research_range-research_start + 1), 3);  % various - location of all fitting points for RANSAC-fitting
                mappingSeqVal_cor = zeros(seq_num*(research_range-research_start + 1), 1);  % various - mapping metrics
    
                % get data into local various
                for count_s = 1:seq_num
                    fittingPoints_cor((count_s-1)*(research_range-research_start + 1)+1:count_s*(research_range-research_start + 1),:) ...
                        = result_cor(research_start:research_range,:,count_s,count_i);

                    mappingSeqVal_cor((count_s-1)*(research_range-research_start + 1)+1:count_s*(research_range-research_start + 1)) ...
                        = value_cor(research_start:research_range,:,count_s,count_i);
                end

                % send data into global various
                mapping_cor(:,:,count_t,count_l,count_i,count_it) = mappingSeqVal_cor;

                [line_cor, IDS_cor] = RANSAC3D(fittingPoints_cor);
%                 InPoints = fittingPoints_cor(IDS_cor,:);

                fittingmapping_cor(1:sum(IDS_cor),:,count_t,count_l,count_i,count_it) = ...
                    mappingSeqVal_cor(IDS_cor,:);

                uvec_cor  = line_cor(1,:);
                start_cor = line_cor(2,:);

                start_zeros = start_cor - start_cor(1)*uvec_cor;            % points| x = 0;
                end_ten     = start_cor - (start_cor(1) - seq_num) * uvec_cor;   % Points| x = 80 mm;

                cal_cor_t(count_l,count_t,count_i,count_it) = (start_zeros(2) + end_ten(2))/2;
                cal_cor_l(count_l,count_t,count_i,count_it) = (start_zeros(3) + end_ten(3))/2;

                click_ratio_cor(count_l,count_t,count_i,count_it) = sum(IDS_cor)/length(IDS_cor);
                
                %%%%%%%%%%%%%%%%%%% dtw evalution
                fittingPoints_dtw = zeros(seq_num*(research_range-research_start + 1),3);
                mappingSeqVal_dtw = zeros(seq_num*(research_range-research_start + 1),1);   

                for count_s = 1:seq_num
                    fittingPoints_dtw((count_s-1)*(research_range-research_start + 1)+1:count_s*(research_range-research_start + 1),:) ...
                        = result_dtw(research_start:research_range,:,count_s,count_i);
                    
                    mappingSeqVal_dtw((count_s-1)*(research_range-research_start + 1)+1:count_s*(research_range-research_start + 1)) = ...
                        value_dtw(research_start:research_range,:,count_s,count_i);
                end
    
                mapping_dtw(:,:,count_t,count_l,count_i,count_it) = mappingSeqVal_dtw;
    
                [line_dtw, IDS_dtw] = RANSAC3D(fittingPoints_dtw);
%                 InPoints = fittingPoints_dtw(IDS_dtw,:);
    
                fittingmapping_dtw(1:sum(IDS_dtw),:,count_t,count_l,count_i,count_it) = ...
                    mappingSeqVal_dtw(IDS_dtw,:);
                
                uvec_dtw  = line_dtw(1,:);     % fitting line - vector
                start_dtw = line_dtw(2,:);     % fitting line - start point
        
                start_zeros = start_dtw - start_dtw(1)*uvec_dtw;            % points| x = 0;
                end_ten     = start_dtw - (start_dtw(1) - seq_num) * uvec_dtw;   % Points| x = 10 mm;
                
                cal_dtw_t(count_l,count_t,count_i,count_it) = (start_zeros(2) + end_ten(2))/2;
                cal_dtw_l(count_l,count_t,count_i,count_it) = (start_zeros(3) + end_ten(3))/2;

                click_ratio_dtw(count_l,count_t,count_i,count_it) = sum(IDS_dtw)/length(IDS_dtw);
            end   % count_it   
        end  % count_i (1:5)
    end  % count_t (1:traj_num)
end  % count_l (1:layer_num)

%% data fusion for different components
for i = 1:5
    it_cal_cor_t(:,:,i) = GravityChoose(cal_cor_t(:,:,:,i));
    it_cal_cor_l(:,:,i) = GravityChoose(cal_cor_l(:,:,:,i));

    it_cal_dtw_t(:,:,i) = GravityChoose(cal_dtw_t(:,:,:,i));
    it_cal_dtw_l(:,:,i) = GravityChoose(cal_dtw_l(:,:,:,i));
end

for i = 1:5
    figure()
    subplot(2,2,1);
    bar3(it_cal_cor_t(:,:,i) - gt_t)
    title(strcat("cor_t",int2str(i)))
    subplot(2,2,2);
    bar3(it_cal_cor_l(:,:,i) - gt_l)
    title(strcat("cor_l",int2str(i)))
    subplot(2,2,3);
    bar3(it_cal_dtw_t(:,:,i) - gt_t)
    title(strcat("dtw_t",int2str(i)))
    subplot(2,2,4);
    bar3(it_cal_dtw_l(:,:,i) - gt_l)
    title(strcat("dtw_l",int2str(i)))
end

for i = 1:5
    [it2_cal_cor_t(:,:,i),it2_cal_cor_l(:,:,i)] = GravityChooseDOUBLE(cal_cor_t(:,:,:,i),cal_cor_l(:,:,:,i));
    [it2_cal_dtw_t(:,:,i),it2_cal_dtw_l(:,:,i)] = GravityChooseDOUBLE(cal_dtw_t(:,:,:,i),cal_dtw_l(:,:,:,i));
end

for i = 1:5
    figure()
    subplot(2,2,1);
    bar3(it2_cal_cor_t(:,:,i) - gt_t)
    title(strcat("cor_t",int2str(i)))
    subplot(2,2,2);
    bar3(it2_cal_cor_l(:,:,i) - gt_l)
    title(strcat("cor_l",int2str(i)))
    subplot(2,2,3);
    bar3(it2_cal_dtw_t(:,:,i) - gt_t)
    title(strcat("dtw_t",int2str(i)))
    subplot(2,2,4);
    bar3(it2_cal_dtw_l(:,:,i) - gt_l)
    title(strcat("dtw_l",int2str(i)))
end

%% show detail result
click_ratio_cor
click_ratio_dtw

% %% shown the gray-picture
% figure()
% for i = 1:layer_num
%     for j = 1:traj_num
%         subplot(layer_num, traj_num, (i-1)*traj_num + j)
%         plot(result_m(5,:,j,i))
%         hold on
%         plot(sim_result(5,:,j,i))
%     end
% end

%% shown the dtw-distance or corrcoef

cor_all_mean = [];
cor_all_max = [];
cor_all_min = [];
dtw_all_mean = [];
dtw_all_max = [];
dtw_all_min = [];

cor_fit_mean = [];
cor_fit_max = [];
cor_fit_min = [];
dtw_fit_mean = [];
dtw_fit_max = [];
dtw_fit_min = [];

for count_it = 1:5
    for count_i = 1:5
        for count_l = 1:layer_num
            for count_t = 1:traj_num
    %             cor_all_mean(count_l,count_t,count_i,count_it) = mean(mapping_cor(:,:,count_t,count_l,count_i,count_it));
    %             cor_all_max(count_l,count_t,count_i,count_it) = max(mapping_cor(:,:,count_t,count_l,count_i,count_it));
    %             cor_all_min(count_l,count_t,count_i,count_it) = min(mapping_cor(:,:,count_t,count_l,count_i,count_it));
                
                dtw_all_mean(count_l,count_t,count_i,count_it) = mean(mapping_dtw(:,:,count_t,count_l,count_i,count_it));
                dtw_all_max(count_l,count_t,count_i,count_it) = max(mapping_dtw(:,:,count_t,count_l,count_i,count_it));
                dtw_all_min(count_l,count_t,count_i,count_it) = min(mapping_dtw(:,:,count_t,count_l,count_i,count_it));
                
    %             cor_fit_mean(count_l,count_t,count_i,count_it) = mean(fittingmapping_cor(:,:,count_t,count_l,count_i,count_it));
    %             cor_fit_max(count_l,count_t,count_i,count_it) = max(fittingmapping_cor(:,:,count_t,count_l,count_i,count_it));
    %             cor_fit_min(count_l,count_t,count_i,count_it) = min(fittingmapping_cor(:,:,count_t,count_l,count_i,count_it));
    %             
                dtw_fit_mean(count_l,count_t,count_i,count_it) = mean(fittingmapping_dtw(:,:,count_t,count_l,count_i,count_it));
                dtw_fit_max(count_l,count_t,count_i,count_it) = max(fittingmapping_dtw(:,:,count_t,count_l,count_i,count_it));
                dtw_fit_min(count_l,count_t,count_i,count_it) = min(fittingmapping_dtw(:,:,count_t,count_l,count_i,count_it));
            end
        end
    
    %     nameFile_cor = strcat("corr-",label(count_i));
    %     figure(nameFile_cor)
    %     subplot(3,2,1)
    %     bar3(cor_all_mean(count_i,count_it))
    %     title('All mean')
    %     subplot(3,2,2)
    %     bar3(cor_fit_mean(count_i,count_it))
    %     title('Fitting mean')
    %     subplot(3,2,3)
    %     bar3(cor_all_max(count_i,count_it))
    %     title('All max')
    %     subplot(3,2,4)
    %     bar3(cor_fit_max(count_i,count_it))
    %     title('Fitting max')
    %     subplot(3,2,5)
    %     bar3(cor_all_min(count_i,count_it))
    %     title('All min')
    %     subplot(3,2,6)
    %     bar3(cor_fit_min(count_i,count_it))
    %     title('All')
    
%         nameFile_dtw = strcat("dtw-",label(count_i));
%         figure()
%         subplot(3,2,1)
%         bar3(dtw_all_mean(count_i,count_it))
%         title('All mean')
%         subplot(3,2,2)
%         bar3(dtw_fit_mean(count_i,count_it))
%         title('Fitting mean')
%         subplot(3,2,3)
%         bar3(dtw_all_max(count_i,count_it))
%         title('All max')
%         subplot(3,2,4)
%         bar3(dtw_fit_max(count_i,count_it))
%         title('Fitting max')
%         subplot(3,2,5)
%         bar3(dtw_all_min(count_i,count_it))
%         title('All min')
%         subplot(3,2,6)
%         bar3(dtw_fit_min(count_i,count_it))
%         title('Fitting min')
    
        channel_label = strcat("dtw: ",label(count_i));
        interaction_label = strcat(", it: ",int2str(count_it));
        nameFile_dtw = strcat(channel_label,interaction_label);
        figure()
        subplot(2,1,1)
        bar3(dtw_all_mean(count_i,count_it))
        title(strcat(nameFile_dtw,"- all mean"))
        subplot(2,1,2)
        bar3(dtw_fit_mean(count_i,count_it))
        title(strcat(nameFile_dtw,"- fitting mean"))

        % save figures - statics
%         savefig(figure(nameFile_cor),strcat(dirName,strcat(nameFile_cor,'.fig')));
%         savefig(figure(nameFile_dtw),strcat(dirName,strcat(nameFile_dtw,'.fig')));
    end  % count_i
end % count_it % shown the statistics result

%% shown location error

figure()
subplot(2,3,1)
bar3(cal_cort_t)
title("CorrCoef-trajectory")

subplot(2,3,2)
bar3(cal_cort_l)
title("CorrCoef-layer")

subplot(2,3,3)
bar3(dist-mapping-cor)
title("CorrCoef-Mapping-error")

subplot(2,3,4)
bar3(cal_dtw_t)
title("DTW-trajectory")

subplot(2,3,5)
bar3(cal_dtw_l)
title("DTW-layer")

subplot(2,3,6)
bar3(dist-mapping-dtw)
title("DTW-Mapping-error")




% % save figures - location
% savefig(figure("Location"),strcat(dirName,"Location.fig"))
% savefig(figure("Location-error"),strcat(dirName,"Location-error.fig"))


% save analysis_dtw.mat fittingmapping_cor fittingmapping_dtw mapping_cor mapping_dtw



%% %%%%%%%%%%%%%%%%%%

% - function
%      FindMappingPath                     -- VERSION 1.0
%      Add color to identify               -- VERSION 1.1 (deleted version in 3.0)
%      get the corrcoef / dtw-distance.    -- VERSION 2.0
%      location error analysis             -- VERSION 3.0
%      physical error evaluation           -- VERSION 3.0
%      test interaction efficiency         -- VERSION 3.0




