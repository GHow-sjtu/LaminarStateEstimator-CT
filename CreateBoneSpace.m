function volume = CreateBoneSpace(space, res, layer)
% author:   Jihao Liu
% data:     July 6, 2022
% function: Create a simulation environment to descript the bone in CT
%            image.
% layer = 2, or 3
%
workSpace = round(space./res);
if layer == 2
    
    z1 = round(workSpace(3)/2);
    z2 = workSpace(3) - z1;
    space1 = [workSpace(1:2) z1];
    space2 = [workSpace(1:2) z2];
    sw1 = rand(space1)*600 + 800;
    sw2 = rand(space2)*600 + 200;
    volume(:,:,1:z1) = sw1;
    volume(:,:,z1+1:z1+z2) = sw2;

    mean_density = sum(sum(sum(volume(:,:,z1-2:z1+2))))/(workSpace(1)*workSpace(2)*5);

    for i = z1-1:z1+1
        volume(:,:,i) = randn(workSpace(1),workSpace(2))*200 + mean_density;
    end

else
    z1 = round(workSpace(3)/4);
    z2 = z1*2;
    z3 = workSpace(3) - z1 - z2;

    space1 = [workSpace(1:2) z1];
    space2 = [workSpace(1:2) z2];
    space3 = [workSpace(1:2) z3];
    
    sw1 = rand(space1)*600 + 800;
    sw2 = rand(space2)*600 + 200;
    sw3 = rand(space3)*600 + 800;
    
    volume(:,:,1:z1) = sw1;
    volume(:,:,z1+1:z1+z2) = sw2;
    volume(:,:,z1+z2+1:z1+z2+z3) = sw3;

    mean_density_1 = sum(sum(sum(volume(:,:,z1-2:z1+2))))/(workSpace(1)*workSpace(2)*5);

    for i = z1-1:z1+1
        volume(:,:,i) = randn(workSpace(1),workSpace(2))*200 + mean_density_1;
    end

    mean_density_2 = sum(sum(sum(volume(:,:,z1+z2-2:z1+z2+2))))/(workSpace(1)*workSpace(2)*5);

    for i = z1+z2-1:z1+z2+1
        volume(:,:,i) = randn(workSpace(1),workSpace(2))*200 + mean_density_2;
    end
    
end


end