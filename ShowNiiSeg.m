function [loc, axis_space] = ShowNiiSeg(nii_file, label)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% author: Jihao Liu
% date:   June 2022
% function: segement a CT image in the file of NII by the label number.
% return: the coordination
%
%
%% head for testing this function
% data1 = load_nii('verse620_CT-iso.nii');
% data2 = load_nii('sub-verse620_dir-iso_seg-vert_msk.nii');
% data3 = load_nii('verse620_lam.nii');

img = nii_file.img;
res = nii_file.hdr.dime.pixdim(2:4);
[~,~,num] = size(img);
temp = [];
for i = 1:num
    [loc_x, loc_y] = find(img(:,:,i) == label);
    tmp_loc= [loc_x*res(1), loc_y*res(2), i*res(3)*ones(size(loc_x))];
    temp = [temp; tmp_loc];
end
% 
% [loc_x, loc_y, loc_z] = find(img == label);
% loc = [loc_x, loc_y, loc_z];

plot3(temp(:,1), temp(:,2), temp(:,3), '*');
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');

zoom = [min(temp(:,1)) max(temp(:,1)) min(temp(:,2)) max(temp(:,2)) min(temp(:,3)) max(temp(:,3))];

% return parameter
axis_space = zoom;
loc  = temp;
end