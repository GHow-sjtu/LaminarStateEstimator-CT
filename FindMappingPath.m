function loc_seq = FindMappingPath(v, m3, seq_num)

% author: Jihao Liu
% date:   July 19, 2022
% purpose: Find the mapping path by DTW algorithm

v_len = length(v);
% [len, num] = size(v);
% len: length of the vector, v
% num: the number of the dimension of the vector, v

[~, m_len, traj_num, layer_num] = size(m3);

num_seq_v = floor(v_len/seq_num);
num_seq_m = floor(m_len/seq_num);

% process the real-time data flow, 4-d or 6-d;
pre_v = zeros(5,seq_num,num_seq_v);

for v_i = 1:num_seq_v
    pre_v(:,:,v_i) = v(:,(v_i-1)*seq_num+1:v_i*seq_num);
end


% process the prediction results, 4-d or 6-d;
pre_m3 = zeros(5,seq_num,num_seq_m,traj_num,layer_num);

for m_i = 1:layer_num
    for m_j = 1:traj_num
        for m_k = 1:num_seq_m
            pre_m3(:,:,m_k,m_j,m_i) = m3(:,(m_k-1)*seq_num+1:m_k*seq_num,m_j,m_i);
        end
    end
end

% label_set = {'Fx','Fy','Fz','Mz','Gray'};


cor_res = zeros(num_seq_m,traj_num,layer_num,num_seq_v,5);
dtw_res = zeros(num_seq_m,traj_num,layer_num,num_seq_v,5);
% [layer_num, traj_num, seq_num]


for count_item = 1:5
    for count_seq = 1:num_seq_v
        pre_seq = pre_v(count_item,:,count_seq);

        for count_l=1:layer_num
            for count_t = 1:traj_num
                for count_s = 1:num_seq_m
                     target_seq = pre_m3(count_item,:,count_s,count_t,count_l);
                     
                     cor_temp = corrcoef(pre_seq,target_seq);
                     cor_res(count_s,count_t,count_l,count_seq,count_item) = cor_temp(2);
                     dtw_res(count_s,count_t,count_l,count_seq,count_item) = dtw(pre_seq,target_seq);
                end  % count_s
            end  % count_t
        end  % count_l
    end  % count_seq
end  % count_item

% loc_seq.CoM = cor_res;
% loc_seq.DwM = dtw_res;

result_cor = zeros(5,3,num_seq_v,5);
result_dtw = zeros(5,3,num_seq_v,5);
result_corR = zeros(5,1,num_seq_v,5);
result_dtwR = zeros(5,1,num_seq_v,5);

scale_index = [0 num_seq_m num_seq_m*traj_num];

for count_item = 1:5
    for count_seq = 1:num_seq_v
        temp_cor = cor_res(:,:,:,count_seq,count_item);
        temp_dtw = dtw_res(:,:,:,count_seq,count_item);
        
        for count_c = 1:5
            temp_loc = find(temp_cor == max(max(max(temp_cor))));
            temp_loc = temp_loc(1);  % only get the first item
            result_cor(count_c,:,count_seq, count_item) = index2v(temp_loc,scale_index);
            result_corR(count_c,:,count_seq,count_item) = temp_cor(temp_loc);
            temp_cor(temp_loc) = realmin;

            temp_loc = find(temp_dtw == min(min(min(temp_dtw))));
            temp_loc = temp_loc(1);  % only get the first item
            result_dtw(count_c,:,count_seq, count_item) = index2v(temp_loc,scale_index);
            result_dtwR(count_c,:,count_seq, count_item) = temp_dtw(temp_loc);
            temp_dtw(temp_loc) = realmax;
        end  % count_c
    end  % count_seq
end  % count_item

loc_seq.resC = result_cor;    % loc 
loc_seq.resD = result_dtw;    % loc
loc_seq.resCR = result_corR;  % value@loc
loc_seq.resDR = result_dtwR;  % value@loc

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
function v = index2v(order,scale_index)
% transform the order to the index of N-dimension matrix
N = length(scale_index);
v = zeros(1,N);
for i=N:-1:2
    v(i) = floor(order/scale_index(i))+1;
    order = mod(order,scale_index(i));
    if order == 0
        v(i) = v(i)-1;
        order = scale_index(i);
    end
end
v(1) = order;
end            % function index2v
