function grayVolume = CreateGray(layerNum, inclineAngle, grayRange, res, workSpace)
% author:   Jihao Liu
% data:     July 6, 2022
% function: Create a virtual CT images filled with some gray blocks
% 
% @ layerNum : number of gray blocks
% @ inclineAngle : inclinal angle of every block
% @
% @ grayRange : the range of gray value, a vector with two elements
% @ res : reslution of one voxel
% @ workSpace : the size of the whole CT images

% lengthNum = length(layerNum);
% lengthAngle = length(inclineAngle);
% lengthRange = length(grayRange);

%%% res = [ 0.25,    0.25,     1.0];
%%%        x-axis   y-axis   z-axis

%%% workSpace = [   10,      10,      100];  % unit:  mm
%%%              x-axis   y-axis    z-axis


maxAngle = atan(workSpace(3)/4);

if length(grayRange) == 2
    gray_layers = linspace(grayRange(1), grayRange(2), layerNum);
else
    if length(grayRange) ~= layerNum
        error('Error: the number of layer is incompatible with grayRange !');
    else
        gray_layers = grayRange;
    end
end

layerGray = zeros(1, layerNum +1);

for i= 1:layerNum
    layerGray(i+1) = gray_layers(i);
end

if length(inclineAngle) == layerNum
    gray_angle = inclineAngle;
elseif (length(inclineAngle) == 1)
    gray_angle = ones(1,layerNum)*inclineAngle;
else
    error('Error: the number of angle is incompatible with layers !');
end

if (max(inclineAngle) > pi/2)
    error('Error: Pre-set incline angle should be in the format of rad !');
end

for i = 1:length(gray_angle)
    if gray_angle(i) > maxAngle
        warning('The angle has been reduced into maxAngle about 87.70 degree !');
        gray_angle(i) = maxAngle;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%
%%% layerGray: the gray value of every layer
%%% gray_angle:  the incline angle of every layer refering to the world
%%% coordinate
%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

count_x = ceil(workSpace(1) / (res(1)));   % the number of pixel in the x-axis
count_y = ceil(workSpace(2) / (res(2)));   % the number of pixel in the y-axis
count_z = ceil(workSpace(3) / (res(3)));   % the number of pixel in the z-axis

max_setAngle = max(gray_angle);           % find the max incline angle among the pre-set parameters
max_allowAngle = atan(workSpace(3)/layerNum/4);   % max allowable incline angle in terms of the thickness.

% if max_setAngle > max_allowAngle
%     error('Number of layers is incompatible, because the thicker is not !')
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%
%%%            working 
%%%
% disp('Virtual workspace of 10 mm * 10 mm * 100 mm is generating with specified gray ...');

grayVolume = zeros(count_x, count_y, count_z);
% disp('The workspace is built!');

LayerPoints = round(linspace(1, count_z, layerNum + 1));
MiddlePoints = zeros(1, layerNum);

for i = 1:layerNum
    MiddlePoints(i) = round( (LayerPoints(i) + LayerPoints(i+1))/2 );
end

Z_Unit = round(count_z/layerNum);            % number of pixels of every layer in point clouse

for k = 1:layerNum

    X_Work = round( (Z_Unit * res(3) / tan(gray_angle(k)) / res(1)) / 2);   % half
    X_Unit = round( (res(3) / tan(gray_angle(k)) / res(1)) / 2);            % half

    if X_Unit > (count_x/2)
        % one layer works as the boundary plane.
        warning('The pxiel number of every layer is over x-axis !');
        grayVolume(:,:,LayerPoints(k):(MiddlePoints(k)-1)) = ones(count_x,count_y, MiddlePoints(k)-LayerPoints(k)) * layerGray(k);
        grayVolume(:,:,MiddlePoints(k):LayerPoints(k+1)) = ones(count_x,count_y, LayerPoints(k+1)-MiddlePoints(k)+1) * layerGray(k+1);
    elseif X_Work > (count_x/2)
        % there exist some plane with the previous gray value in one working layer.
        validNum = ceil( workSpace(1) * tan(gray_angle(k)) / res(3) );
        difNum = round(validNum / 2);
        topNum = MiddlePoints(k) - difNum;
        botNum = MiddlePoints(k) + difNum;

        % Fill with the top volume with previous gray .
        grayVolume(:,:,LayerPoints(k):botNum) = ones(count_x,count_y, botNum-LayerPoints(k)+1) * layerGray(k);

        % Fill from the bottom plane to next layer with gray of this layer
        if (k < layerNum)
            grayVolume(:,:,(botNum+1):MiddlePoints(k+1)) = ones(count_x, count_y, MiddlePoints(k+1) - botNum) * layerGray(k+1);
        else
            grayVolume(:,:,(botNum+1):LayerPoints(k+1)) = ones(count_x, count_y, LayerPoints(k+1) - botNum) * layerGray(k+1);
        end


        % Fill with the specified slop volume with gray of this layer.
        for i = topNum:botNum
            local_num = i + 1 - topNum;
            local_tol = round( local_num * res(3) / tan(gray_angle(k)) / res(1) );
            if local_tol < count_x
                grayVolume(1:local_tol, :, i) = ones(local_tol, count_y) * layerGray(k+1);
            else
                grayVolume(:, :, i) = ones(count_x, count_y) * layerGray(k+1);
            end
        end
    else
        % File the volume with the previous gray, firstly
        grayVolume(:,:,(LayerPoints(k)+1):LayerPoints(k+1)) = ones(count_x, count_y, LayerPoints(k+1)-LayerPoints(k)) * layerGray(k+1);
        for j = (LayerPoints(k)+1):LayerPoints(k+1)
            local_middle_x = round(count_x / 2);
            % local_num = j - LayerPoints(k) ;
            local_x = local_middle_x + round( (j - MiddlePoints(k)) * res(3) / tan(gray_angle(k)) / res(1) );
            if local_x < 1
                local_x = 1;
            elseif local_x > count_x
                local_x = count_x;
            end
            grayVolume(1:local_x, :, j) = ones(local_x, count_y) * layerGray(k+1);
        end
    end
end

% show three views of the created ct images 
Show2DimGrayBone(grayVolume);

end