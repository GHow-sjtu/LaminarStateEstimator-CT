function tran = GetRotLoc(m_l,m_t)
% author: Jihao Liu
% date:   Aug. 9, 2022
% function:

interaction_num = 1000;
up_error = 2;
best_num = 30;

[num_l,num_t] = size(m_l);

ref_l = [1:num_l]'*ones(1,num_t);
ref_t = ones(num_l,1)*[1:num_t];

min_err = realmax;
opt_res = [];

count_t = 0;    % if the min-error repeats 3 time, we can stop.
for count_int = 1: interaction_num

    loc_error = realmax;             % ...
    % 
    loc_init = sort(randperm(num_t*num_l,best_num));

    l_loc = mean(m_l(loc_init) - ref_l(loc_init));
    t_loc = mean(m_t(loc_init) - ref_t(loc_init));

    while(1)
        % calculate the error
        error_test = sqrt( (m_l - ref_l - l_loc).^2 + (m_t - ref_t - t_loc).^2 );
        
        % find the location in terms of the error
        loc_temp = find(error_test <= up_error);
        
        % there exists no.
        if isempty(loc_temp)
            break;
        end

        % update the error
        error_m = mean(error_test(loc_temp));
    
        if error_m < loc_error
            loc_error = error_m;
        else
            % this loop can stop when the error is 
            break;
        end

        % the procedure can stop when the research range is not changed.
        if loc_temp == loc_init
            break;
        end

        % update the loc-value
        l_loc = mean(m_l(loc_temp) - ref_l(loc_temp));
        t_loc = mean(m_t(loc_temp) - ref_t(loc_temp));
    end  % inner - loop

    % update the error - every optimal research
    if loc_error < min_err
        min_err = loc_error;
        opt_res = [l_loc, t_loc];
    elseif loc_error == min_err
        count_t = count_t + 1;
        if count_t == 3
%             count_int
            break
        end
    end

end  % interaction

% return the result
tran = opt_res;

end


%  function:
%                  -- Version 1.0