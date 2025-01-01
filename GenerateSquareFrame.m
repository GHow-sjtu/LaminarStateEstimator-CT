function ptClouse = GenerateSquareFrame(space, tran, varargin)
Len = space(1);
Wid = space(2);
Hei = space(3);

if isempty(varargin)
    color = abs(rand(1,3));
else
    if length(varargin{1}) == 3
        color = varargin{1};
    else
        color = abs(rand(1,3));
    end
end

L0_x = -Len/2:0.1:Len/2;
W0_y = -Wid/2:0.1:Wid/2;
H0_z = -Hei/2:0.1:Hei/2;

L1 = [L0_x',  Wid/2*ones(size(L0_x')),  Hei/2*ones(size(L0_x')), ones(size(L0_x'))];
L2 = [L0_x', -Wid/2*ones(size(L0_x')),  Hei/2*ones(size(L0_x')), ones(size(L0_x'))];
L3 = [L0_x',  Wid/2*ones(size(L0_x')), -Hei/2*ones(size(L0_x')), ones(size(L0_x'))];
L4 = [L0_x', -Wid/2*ones(size(L0_x')), -Hei/2*ones(size(L0_x')), ones(size(L0_x'))];

W1 = [ Len/2*ones(size(W0_y')), W0_y',  Hei/2*ones(size(W0_y')), ones(size(W0_y'))];
W2 = [-Len/2*ones(size(W0_y')), W0_y',  Hei/2*ones(size(W0_y')), ones(size(W0_y'))];
W3 = [ Len/2*ones(size(W0_y')), W0_y', -Hei/2*ones(size(W0_y')), ones(size(W0_y'))];
W4 = [-Len/2*ones(size(W0_y')), W0_y', -Hei/2*ones(size(W0_y')), ones(size(W0_y'))];

H1 = [ Len/2*ones(size(H0_z')),  Wid/2*ones(size(H0_z')), H0_z', ones(size(H0_z'))];
H2 = [-Len/2*ones(size(H0_z')),  Wid/2*ones(size(H0_z')), H0_z', ones(size(H0_z'))];
H3 = [ Len/2*ones(size(H0_z')), -Wid/2*ones(size(H0_z')), H0_z', ones(size(H0_z'))];
H4 = [-Len/2*ones(size(H0_z')), -Wid/2*ones(size(H0_z')), H0_z', ones(size(H0_z'))];

X_axis = 0:0.1:Len/2;
Y_axis = 0:0.1:Wid/2;
Z_axis = 0:0.1:Hei/2;

% x_axis_f = [X_axis; zeros(size(X_axis)); zeros(size(X_axis))]';
% y_axis_f = [zeros(size(Y_axis)); Y_axis; zeros(size(Y_axis))]';
% z_axis_f = [zeros(size(Z_axis)); zeros(size(Z_axis)); Z_axis]';

x_axis_f = X_axis'*[1 0 0];
y_axis_f = Y_axis'*[0 1 0];
z_axis_f = Z_axis'*[0 0 1];

len_x = length(X_axis);
len_y = length(Y_axis);
len_z = length(Z_axis);

temp = [L1;L2;L3;L4;W1;W2;W3;W4;H1;H2;H3;H4];
len_num = length(temp);
% color_frame = ones(len_num,1)*color;
temp = [temp; x_axis_f ones([size(x_axis_f, 1), 1]); y_axis_f ones([size(y_axis_f, 1), 1]); z_axis_f ones([size(z_axis_f, 1), 1])];
R_color = [1 0 0];
G_color = [0 1 0];
B_color = [0 0 1];
COL_W = [ones(len_num,1)*color; ones(len_x,1)*R_color; ones(len_y,1)*G_color; ones(len_z,1)*B_color];

len_num = length(temp);
Loc_pos = zeros(size(temp));
for i = 1:len_num
    Loc_pos(i,:) = tran*temp(i,:)';
end

ptClouse = pointCloud(Loc_pos(:,1:3),'color',COL_W);
end