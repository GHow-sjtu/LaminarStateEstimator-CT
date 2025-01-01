function loc = SampleNiiSeg(nii_file, label, sampleRatio)

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

tol_num = length(temp);

% down-sampling by using the samplingRatio
tmp_loc = [];
for i = 1:sampleRatio:tol_num
    tmp_loc = [tmp_loc; temp(i,:)];
end

loc  = tmp_loc;
end