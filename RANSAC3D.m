function [model_res, inlierIdx_res] = RANSAC3D(Points)
% author: Jihao
% date:   July 27, 2022
% description: this function is used to fit a space line by eliminate
% noisy.

% Given:
% data - A set of observations
% model- A model to explain observed data points
% n - Minimum number of data points required to estimate model parameters.
% k - Maximum number of iterations allowd in the algorithm
% t - Threshold value to determine data points that are fit well by model.
% d - Number of close data points required to assert that a model fits well
%     to data.

% Return:
%   bestFit - model parameters which best fit the data (or null if no good
%   model is found)


sampleSize = 2;     % the number of the sampling points everytime, 2 for the line fitting
maxDistance = 0.5;  % the maximum distance between the inner point and the model
fitLineFcn = @(Points) FitMethod(Points);

evalLineFcn = @(model, Points) EvaluateFit(model, Points);  % distance evaluation function

[modelRANSAC, inlierIdx] = ransac(Points, fitLineFcn, evalLineFcn, sampleSize, maxDistance,'MaxNumTrials',2000);
% 
% for i = 1:num
%     temp = -i*modelRANSAC(1,:) + start;
% 
%     Points(i+1,:) = temp;
% end
% hold on
% plot3(Points(:,1),Points(:,2), Points(:,3),'*')


if modelRANSAC(1) < 0
    model_res = [-modelRANSAC(1,:);modelRANSAC(2,:)];
else
    model_res = modelRANSAC;
end

inlierIdx_res = inlierIdx;

end