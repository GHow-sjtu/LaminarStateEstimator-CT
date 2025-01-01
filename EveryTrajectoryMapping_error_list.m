clear;clc;close all;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%
% author: Jihao Liu
% date:   June, 2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%%   Load data from
load result_new0817.mat

% save result_new.mat result_m sim_result
[~,~,traj_num,layer_num] = size(result_m);

label = {'Fx','Fy','Fz','Mz','Gray'};

% to record the mapping ratio
click_ratio_cor = zeros(layer_num,traj_num,5);
click_ratio_dtw = zeros(layer_num,traj_num,5);

%% % post - show the mapping fitting trajectory
% test_one  = result_m(:,:,1,1);

error_list_cor = [];
error_list_dtw = [];
zeros_cor = [];
zeros_dtw = [];

% 
% fittingLoc_cor = [];
% fittingLoc_dtw = [];
% fittingvec_cor = [];
% fittingvec_dtw = [];

mapping_cor = [];
mapping_dtw = [];

fittingmapping_cor = [];
fittingmapping_dtw = [];

color_set = hsv(layer_num * traj_num);

index = 1;  % for color

% include-angle of fitting line.
angle_allow  = pi/8;
% the number of points for fitting.
research_range = 1;

% ground_true
gt_t = ones(layer_num,1)*[1:traj_num];
gt_l = [1:layer_num]'*ones(1,traj_num);

cal_cor_t = NaN([size(gt_t) 5]);
cal_cor_l = NaN([size(gt_l) 5]);

cal_dtw_t = NaN([size(gt_t) 5]);
cal_dtw_l = NaN([size(gt_l) 5]);

dist_mapping_cor = NaN(layer_num,traj_num,5);
dist_mapping_dtw = NaN(layer_num,traj_num,5);

MappingNum_cor = zeros(layer_num,traj_num);
MappingNum_dtw = zeros(layer_num,traj_num);

for count_l = 1:layer_num
    for count_t = 1:traj_num
        test_one = result_m(:,:,count_t, count_l);
        result_one = FindMappingPath(test_one, result_m, 15);

        result_cor = result_one.resC;   % the location of mapping points based on CORRCOEF
        result_dtw = result_one.resD;   % the location of mapping points based on DTW
        value_cor = result_one.resCR;   % the value vs. loc
        value_dtw = result_one.resDR;   % the value vs. loc

        [~,~,seq_num,~] = size(result_cor);
        
        for count_i = 1:5
            figure(count_i)
            
            fittingPoints_cor = zeros(seq_num*research_range,3);
            mappingSeqVal_cor = zeros(seq_num*research_range,1);

            for count_s = 1:seq_num
                fittingPoints_cor((count_s-1)*research_range+1:count_s*research_range,:) ...
                    = result_cor(1:research_range,:,count_s,count_i);
                
                mappingSeqVal_cor((count_s-1)*research_range+1:count_s*research_range) = ...
                    value_cor(1:research_range,:,count_s,count_i);

            end
            mapping_cor(:,:,count_t,count_l,count_i) = mappingSeqVal_cor;

            [line_cor, IDS_cor] = RANSAC3D(fittingPoints_cor);
            InPoints = fittingPoints_cor(IDS_cor,:);
            
            MappingNum_cor(count_l,count_t) = sum(IDS_cor);
            
            fittingmapping_cor(1:sum(IDS_cor),:,count_t,count_l,count_i) = ...
                mappingSeqVal_cor(IDS_cor,:);

            start_cor = line_cor(2,:);
            uvec_cor  = line_cor(1,:);
            % start_cor = zeros(x)  + vecTor * length.
            % where, length = (start_cor - zeros(x))/vectTor
            if (acos(dot([1 0 0],uvec_cor))< angle_allow)
                plot3(InPoints(:,1),InPoints(:,2),InPoints(:,3),'*','color',color_set(index,:))
                hold on
                start_zeros = start_cor - start_cor(1)*uvec_cor;            % points| x = 0;
                end_ten     = start_cor - (start_cor(1) - seq_num) * uvec_cor;   % Points| x = 80 mm;

                temp_t = (start_zeros(2) + end_ten(2))/2;
                temp_l = (start_zeros(3) + end_ten(3))/2;
                cal_cor_t(count_l,count_t,count_i) = temp_t;
                cal_cor_l(count_l,count_t,count_i) = temp_l;
                
                dist = sqrt((temp_t - gt_t(count_l,count_t))^2 ...
                    + (temp_l - gt_l(count_l,count_t))^2);
                               
                dist_mapping_cor(count_l,count_t,count_i) = dist;

%                 fittingLoc_cor = [fittingLoc_cor; start_zeros];
%                 fittingvec_cor = [fittingvec_cor; uvec_cor];
                linePoints_cor = [start_zeros; end_ten];
                plot3(linePoints_cor(:,1),linePoints_cor(:,2),linePoints_cor(:,3),'-','color',color_set(index,:));
                hold on
                click_ratio_cor(count_l,count_t,count_i) = sum(IDS_cor)/length(IDS_cor);
            else
                error_list_cor = [error_list_cor; count_i count_l count_t];
                click_ratio_cor(count_l,count_t,count_i) = 0;
            end
        end

        for count_i = 1:5
            figure(count_i+5)
            
            fittingPoints_dtw = zeros(seq_num*research_range,3);
            mappingSeqVal_dtw = zeros(seq_num*research_range,1);


            for count_s = 1:seq_num
                fittingPoints_dtw((count_s-1)*research_range+1:count_s*research_range,:) ...
                    = result_dtw(1:research_range,:,count_s,count_i);
                
                mappingSeqVal_dtw((count_s-1)*research_range+1:count_s*research_range) = ...
                    value_dtw(1:research_range,:,count_s,count_i);
            end

            mapping_dtw(:,:,count_t,count_l,count_i) = mappingSeqVal_dtw;

            [line_dtw, IDS_dtw] = RANSAC3D(fittingPoints_dtw);
            InPoints = fittingPoints_dtw(IDS_dtw,:);
            
            MappingNum_dtw(count_l,count_t) = sum(IDS_dtw);

            fittingmapping_dtw(1:sum(IDS_dtw),:,count_t,count_l,count_i) = ...
                mappingSeqVal_dtw(IDS_dtw,:);
            
            start_dtw = line_dtw(2,:);
            uvec_dtw  = line_dtw(1,:);
            % start_dtw = zeros(x)  + vecTor * length.
            % where, length = (start_cor - zeros(x))/vectTor

            if (acos(dot([1 0 0],uvec_dtw))< angle_allow)
                plot3(InPoints(:,1),InPoints(:,2),InPoints(:,3),'*','color',color_set(index,:))
                hold on
                start_zeros = start_dtw - start_dtw(1)*uvec_dtw;            % points| x = 0;
                end_ten     = start_dtw - (start_dtw(1) - seq_num) * uvec_dtw;   % Points| x = 10 mm;
                

                temp_t = (start_zeros(2) + end_ten(2))/2;
                temp_l = (start_zeros(3) + end_ten(3))/2;
                cal_dtw_t(count_l,count_t,count_i) = temp_t;
                cal_dtw_l(count_l,count_t,count_i) = temp_l;

                dist = sqrt((temp_t - gt_t(count_l,count_t))^2 ...
                    + (temp_l - gt_l(count_l,count_t))^2);
                dist_mapping_dtw(count_l,count_t,count_i) = dist;


%                 fittingLoc_dtw = [fittingLoc_dtw; start_zeros];
%                 fittingvec_dtw = [fittingvec_dtw; uvec_dtw];
                linePoints_dtw = [start_zeros; end_ten];
                plot3(linePoints_dtw(:,1),linePoints_dtw(:,2),linePoints_dtw(:,3),'-','color',color_set(index,:));
                hold on
                click_ratio_dtw(count_l,count_t,count_i) = sum(IDS_dtw)/length(IDS_dtw);
            else
                error_list_dtw = [error_list_dtw; count_i count_l count_t];
                click_ratio_dtw(count_l,count_t,count_i) = 0;
            end

        end
        index = index  + 1;
    end
end

% %% save all picture
% pathName = "result-pic-0807-sim-error";
% mkdir(pathName);
% dirName = strcat(pathName,'/');
% for fig_i = 1:10
%     savefig(figure(fig_i),strcat(dirName,strcat(int2str(fig_i),'.fig')))
% end

% %% show result
click_ratio_cor
click_ratio_dtw
% [num_cor,~] = size(error_list_cor)
% [num_dtw,~] = size(error_list_dtw)
% error_list_cor
% error_list_dtw
% zeros_cor
% zeros_dtw

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

% %% shown the dtw-distance or corrcoef
% 
% cor_all_mean = [];
% cor_all_max = [];
% cor_all_min = [];
% dtw_all_mean = [];
% dtw_all_max = [];
% dtw_all_min = [];
% 
% cor_fit_mean = [];
% cor_fit_max = [];
% cor_fit_min = [];
% dtw_fit_mean = [];
% dtw_fit_max = [];
% dtw_fit_min = [];
% 
% for count_i = 1:5
%     for count_l = 1:layer_num
%         for count_t = 1:traj_num
%             cor_all_mean(count_l,count_t) = mean(mapping_cor(:,:,count_t,count_l));
%             cor_all_max(count_l,count_t) = max(mapping_cor(:,:,count_t,count_l));
%             cor_all_min(count_l,count_t) = min(mapping_cor(:,:,count_t,count_l));
%             
%             dtw_all_mean(count_l,count_t) = mean(mapping_dtw(:,:,count_t,count_l));
%             dtw_all_max(count_l,count_t) = max(mapping_dtw(:,:,count_t,count_l));
%             dtw_all_min(count_l,count_t) = min(mapping_dtw(:,:,count_t,count_l));
%             
%             cor_fit_mean(count_l,count_t) = mean(fittingmapping_cor(1:MappingNum_cor(count_l,count_t),:,count_t,count_l));
%             cor_fit_max(count_l,count_t) = max(fittingmapping_cor(1:MappingNum_cor(count_l,count_t),:,count_t,count_l));
%             cor_fit_min(count_l,count_t) = min(fittingmapping_cor(1:MappingNum_cor(count_l,count_t),:,count_t,count_l));
%             
%             dtw_fit_mean(count_l,count_t) = mean(fittingmapping_dtw(1:MappingNum_dtw(count_l,count_t),:,count_t,count_l));
%             dtw_fit_max(count_l,count_t) = max(fittingmapping_dtw(1:MappingNum_dtw(count_l,count_t),:,count_t,count_l));
%             dtw_fit_min(count_l,count_t) = min(fittingmapping_dtw(1:MappingNum_dtw(count_l,count_t),:,count_t,count_l));
%         end
%     end
%     nameFile_cor = strcat("corr-",label(count_i));
%     figure()
%     subplot(3,2,1)
%     bar3(cor_all_mean)
%     title('All mean')
%     subplot(3,2,2)
%     bar3(cor_fit_mean)
%     title('Fitting mean')
%     subplot(3,2,3)
%     bar3(cor_all_max)
%     title('All max')
%     subplot(3,2,4)
%     bar3(cor_fit_max)
%     title('Fitting max')
%     subplot(3,2,5)
%     bar3(cor_all_min)
%     title('All min')
%     subplot(3,2,6)
%     bar3(cor_fit_min)
%     title('Fitting min')
% 
%     nameFile_dtw = strcat("dtw-",label(count_i));
%     figure()
%     subplot(3,2,1)
%     bar3(dtw_all_mean)
%     title('All mean')
%     subplot(3,2,2)
%     bar3(dtw_fit_mean)
%     title('Fitting mean')
%     subplot(3,2,3)
%     bar3(dtw_all_max)
%     title('All max')
%     subplot(3,2,4)
%     bar3(dtw_fit_max)
%     title('Fitting max')
%     subplot(3,2,5)
%     bar3(dtw_all_min)
%     title('All min')
%     subplot(3,2,6)
%     bar3(dtw_fit_min)
%     title('Fitting min')
% end


%% shown location error
for i =1:5
    figure()
    subplot(2,2,1)
    bar3(cal_cor_t(:,:,i))
    title("CorrCoef-trajectory")

    subplot(2,2,2)
    bar3(cal_cor_l(:,:,i))
    title("CorrCoef-layer")

    subplot(2,2,3)
    bar3(cal_dtw_t(:,:,i))
    title("DTW-trajectory")

    subplot(2,2,4)
    bar3(cal_dtw_l(:,:,i))
    title("DTW-layer")


    figure()
    subplot(2,1,1)
    bar3(dist_mapping_cor(:,:,i))
    title("CorrCoef_Mapping_error")
    subplot(2,1,2)
    bar3(dist_mapping_dtw(:,:,i))
    title("DTW_Mapping_error")
end
% 
% savefig(figure("Location"),"Location.fig")
% savefig(figure("Location-error"),"Location-error.fig")
% 
% 
% save analysis_dtw.mat fittingmapping_cor fittingmapping_dtw mapping_cor mapping_dtw

for i = 1:5
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

mkdir('result_selffig')
for i=1:26
    figure(i)
    savefig(strcat(strcat('result_selffig/',int2str(i)),'.fig'))
end


%% %%%%%%%%%%%%%%%%%%

% - function
%      FindMappingPath
%      Add color to identify
%      get the corrcoef / dtw-distance.



