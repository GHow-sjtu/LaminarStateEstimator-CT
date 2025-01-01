function gray = GrayMeanTran(Points, res, Map, TM)

mu = 1.815;       % system parameter

[~, num] = size(Points);


[max_m, max_n, max_l] = size(Map);

temp_gray = zeros(num,1);

R_t = TM(1:3,1:3);
T_t = TM(1:3,4);

for i = 1:num
    temp_point = R_t*Points(1:3,i) + T_t;

    m = round(temp_point(1) / res(1));
    n = round(temp_point(2) / res(2));
    l = round(temp_point(3) / res(3));

    if m<1
        m = 1;
    end
    if n<1
        n = 1;
    end
    if l<1
        l = 1;
    end

    if m>max_m
        m = max_m;
    end
    if n>max_n
        n = max_n;
    end
    if l > max_l
        l = max_l;
    end

    % lammda_sp = getLambda(grayVolume(pix_position_Sp(1),pix_position_Sp(2),pix_position_Sp(3)));
    temp_gray(i) = Map(m,n,l);
    
end

gray = mean(temp_gray);
end