close all;
clear;
clc;
n=load_nii('data\test.nii');
img = n.img;
[size_m, size_n, size_l] = size(img);
p=[];
round(size_m/4)
round(3*size_m/4)
for i = round(size_m/4): round(3*size_m/4)
    for j = round(size_n/4):round(3*size_n/4)
        for k = round(size_l/3):round(2*size_l/3)
            if(img(i,j,k) > 65)
                tmp = [i,j,k];
                p = [p; tmp];
            end
        end
    end
end


q=[];
round(size_m/4)
round(3*size_m/4)
for i = round(size_m/4): round(3*size_m/4)
    for j = round(size_n/4):round(3*size_n/4)
        for k = round(size_l/3):round(2*size_l/3)
            if(img(i,j,k) > 110)
                tmp = [i,j,k];
                q = [q; tmp];
            end
        end
    end
end

save 'data\pcl.m' p q

pcshow([p(:,1)*0.2559 p(:,2)*0.2558 p(:,3)])
pcshow([q(:,1)*0.2559 q(:,2)*0.2558 q(:,3)])

view_nii(n)



