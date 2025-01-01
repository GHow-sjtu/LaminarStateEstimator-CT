function Force = InstanceForceTran(Points, res, Map, R, d_theta, TM)

mu = 1.815;       % system parameter

[~, num] = size(Points);

temp_force = zeros(3,num);
temp_mom   = zeros(1,num);
[max_m, max_n, max_l] = size(Map);

% TM : phisical space.

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
    lammda = getLambda(Map(m,n,l));
    modify = lammda^mu;
    factors = GenerateCoefficients(pi/6,3);
    v_e = [Points(7,i), Points(8,i)]';
    temp = modify * factors * v_e * d_theta;  % temp = [Fr, Fa, Ft]'

    theta = Points(5,i);
    phi   = Points(6,i);

    T_t = [-cos(theta)*cos(phi), -sin(theta)*cos(phi), sin(phi);
           -cos(theta)*sin(phi), -sin(theta)*sin(phi), -cos(phi);
           -sin(theta),          cos(theta),            0];
    temp_F = T_t * temp;
    temp_force(:,i) = temp_F;
    temp_mom(i) = R * [cos(theta)*cos(phi), cos(theta)*sin(phi)] * temp_F(1:2);
end

f = sum(temp_force, 2);
m = sum(temp_mom);


Force = [f;m];
end