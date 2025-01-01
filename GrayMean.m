function gray = GrayMean(Points, res, Map)

mu = 1.815;       % system parameter

[~, num] = size(Points);


[max_m, max_n, max_l] = size(Map);

temp_gray = zeros(num,1);
for i = 1:num
    m = round(Points(1,i) / res(1));
    n  = round(Points(2,i) / res(2));
    l   = round(Points(3,i) / res(3));

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