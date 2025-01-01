clear;clc;close all;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% author: Jihao Liu
% date:   June 2022
% function: Generate the space of the milling planning
%%
% data1 = load_nii('verse620_CT-iso.nii');
data2 = load_nii('verse503_seg.nii');
data3 = load_nii('sub-verse503_L2.nii');
res = data2.hdr.dime.pixdim(2:4);       % resolution

% shwo the point cloud of Lumar 2
figure()
[loc_2,~]=ShowNiiSeg(data2.img, 21);
hold on
[loc_3,~]=ShowNiiSeg(data3.img, 1);
axis square

% the pixel image to the physical image.
Loc_2 = bsxfun(@times, loc_2, res);
Loc_3 = bsxfun(@times, loc_3, res);
% 
% [num_i, ~] = size(loc_2);
% [num_j, ~] = size(loc_3);
% 
% for i = 1:num_i
%     loc_2(i,:) = loc_2(i,:) .* res;
% end
% for j = 1:num_j
%     loc_3(j,:) = loc_3(j,:) .* res;
% end

% temp = loc_2;
% for j = num_j:-1:1
%     [num_i, ~] = size(temp);
%     for  i = num_i:-1:1
%         if loc_2(i,:) == loc_3(j,:)
%             temp(i,:) = [];
% %             loc_3(j,:) = [];
%         end
%     end
% end

num_tol = length(Loc_2);
samLoc_2 = [];
for i =1:11:num_tol
    samLoc_2 = [samLoc_2; Loc_2(i,:)];
end

% show point cloud of the segemented lumar 2 in the actual physical size
ptCloud = pointCloud(Loc_3,'Color',ones(length(Loc_3),1)*[1 0 0]);

% show in different view
figure()
subplot(2,2,1)
pcshow(samLoc_2)
hold on
pcshow(ptCloud)
view([1 0 0])
axis square

subplot(2,2,2)
pcshow(samLoc_2)
hold on
pcshow(ptCloud)
view([0 -1 0])
axis square

subplot(2,2,3)
pcshow(samLoc_2)
hold on
pcshow(ptCloud)
view([0 0 1])
axis square

subplot(2,2,4)
pcshow(samLoc_2)
hold on
pcshow(ptCloud)
view([1 -1 1])
axis square

%% %%%%%%%%%%%%%%%%%%%%%%%%%
%
[nv, cen3P] = GetFixPlane(Loc_3);        % Get the fixing plane of the lamine
R_tran = GenerateRandCoordinate(nv);     % Generate the cordination
Tran_wl = eye(4);
Tran_wl(1:3,1:3) = R_tran;
Tran_wl(1:3,4) = reshape(cen3P,[3,1]);

inv_tran_wl = inv(Tran_wl);
inv_R_tran = inv_tran_wl(1:3,1:3);
inv_t_tran = inv_tran_wl(1:3,4);

Proj_Loc3 = zeros(size(Loc_3));
for i = 1:length(Proj_Loc3)
    Proj_Loc3(i,:) = (inv_R_tran*Loc_3(i,:)' + inv_t_tran)';
end

[an, cen2L] = GetFixLine(Proj_Loc3);

Ax_loc = R_tran * reshape(an,[3,1]);

z  = 0:0.1:10;
tempx = z'*Ax_loc';
Ax_pix = bsxfun(@plus,tempx,cen3P);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% calculate the .. axis
% 
% CoordinationFrame = [Ax_loc; Ay_loc, Az_loc];
% Az_loc =  nv;

R = [Ax_loc, cross(nv', Ax_loc), nv'];
[Frame, tran] = GetFrameSize(Loc_3, R, cen3P);
color = [1 1 1];

Frame = [28.488, 8.8518, 12.237];                       % initial size of the frame
% the center of the workspace works as the origine.
tran  = [-0.090319,  0.87965,   0.46697,   89.098;
         -0.35196,  -0.46682,   0.8113,   102.06;
         -0.93165,   0.091078, -0.35176,  192.89;
               0,   0,         0,           1];

ptc = GenerateSquareFrame(Frame, tran, color);

figure()
pcshow(Loc_2)
hold on
pcshow(ptCloud);
axis square
pcshow(ptc);
view([1 -1 1])
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure()
% pcshow(Loc_2)
% hold on
pcshow(ptCloud);  % red pointcloud.
hold on
axis square
pcshow(ptc);
view([-1 -1 0.5])
hold off

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Make a surgical planning
%  By adjusting the frame manually
%  Rotate
for i = 1:1000
    p = input('Please input rotation X/Y/Z :','s');
    switch p
        case 'x'
            tempr = ROX(-pi/360);    % 0.5 degree
        case 'X'
            tempr = ROX(pi/360);     % 0.5 degree
        case 'y'
            tempr = ROY(-pi/360);    % 0.5 degree
        case 'Y'
            tempr = ROY(pi/360);     % 0.5 degree
        case 'z'
            tempr = ROZ(-pi/360);    % 0.5 degree
        case 'Z'
            tempr = ROZ(pi/360);     % 0.5 degree
        otherwise
            tempr = eye(3);
            break;
    end

    R = R*tempr;
    cen = reshape(tran(1:3,4),[1,3]);
    [Frame, tran] = GetFrameSize(Loc_3, R, cen);
    view([-1 -1 0.5])
    ptc = GenerateSquareFrame(Frame, tran, color);
    Frame
    tran

    pcshow(ptCloud);  % red pointcloud.
    hold on
    axis square
    pcshow(ptc);
    view([-1 -1 0.5])

    hold off
end

%  Translate
for i = 1:1000
    p = input('Please input scale L/W/H, tran X/Y/Z :','s');
    switch p
        case 'x'
            tran(1,4) = tran(1,4) - 0.5;
        case 'X'
            tran(1,4) = tran(1,4) + 0.5;
        case 'y'
            tran(2,4) = tran(2,4) - 0.5;
        case 'Y'
            tran(2,4) = tran(2,4) + 0.5;
        case 'z'
            tran(3,4) = tran(3,4) - 0.5;
        case 'Z'
            tran(3,4) = tran(3,4) + 0.5;
        case 'l'
            Frame(1) = Frame(1) - 1;
        case 'L'
            Frame(1) = Frame(1) + 1;
        case 'w'
            Frame(2) = Frame(2) - 1;
        case 'W'
            Frame(2) = Frame(2) + 1;
        case 'h'
            Frame(3) = Frame(3) - 1;
        case 'H'
            Frame(3) = Frame(3) + 1;
        otherwise
            break;
    end

    color = [1 1 1];
    ptc = GenerateSquareFrame(Frame, tran, color);
    
%     figure(5)
    pcshow(ptCloud);  % red pointcloud.
    hold on
    axis square
    pcshow(ptc);
    view([-1 -1 0.5])

    hold off
end

Frame
tran
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure()
pcshow(samLoc_2)
hold on
pcshow(ptCloud);  % red pointcloud.
hold on
axis square 
pcshow(ptc);
view([-1 -1 0.5])

for i = 1:1000
    p = input('Please input rotation X/Y/Z :','s');
    switch p
        case 'x'
            tempr = ROX(-pi/360);
        case 'X'
            tempr = ROX(pi/360);
        case 'y'
            tempr = ROY(-pi/360);
        case 'Y'
            tempr = ROY(pi/360);
        case 'z'
            tempr = ROZ(-pi/360);
        case 'Z'
            tempr = ROZ(pi/360);
        otherwise
            tempr = eye(3);
            break;
    end

    tran(1:3,1:3) = tran(1:3,1:3)*tempr;
    color = [1 1 1]
    ptc = GenerateSquareFrame(Frame, tran, color);
    Frame
    tran

    pcshow(ptCloud);  % red pointcloud.
    hold on
    axis square
    pcshow(ptc);
    view([-1 -1 0.5])
    hold off
end

Ori_loc = [-Frame/2 1] + [2.0, 2.0, 0.5];
Ori_wld = tran*Ori_loc';
tran_wld = tran;
tran_wld(1:3,4) = Ori_wld(1:3,1);

tran_wld
% tran_wld =

