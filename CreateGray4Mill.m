function grayVolume = CreateGray4Mill(layerNum, inclineAngle, grayRange, res, workSpace)

space0 = workSpace;
space0(1) = workSpace(3);
space0(3) = workSpace(1);
res0      = res;
res0(1)  = res(3);
res0(3)  = res(1);

Volume = CreateGray(layerNum, inclineAngle, grayRange, res0, space0);

[m,n,l] = size(Volume);
grayVolume = zeros(l,n,m);
for i = 1:m
    for j=1:n
        for k = 1:l
            grayVolume(k,j,i) = Volume(i,j,k);
        end
    end
end

Show2DimGrayBone(grayVolume);

end