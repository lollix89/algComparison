function [fittedModel,fittedParam,best_RMSE]= variogram(sCoord, Y, range)
% Variogram compute the experimental variogram and fit it with a spherical
% model
% INPUT 
% sCoord : coordinates of the sampling point
% Y : measured value
% lag,range : parameters used for experimental variogram computation

% OUTPUT 
% fittedModel : Handle of the model used to fit the variogram
% fittedParam : Parameters of the model

%Compute robust Cressie and Hawkins variogram estimator

lag=10;
[lags, varioVal] = kriging.computeRobustVario(range,lag,sCoord,Y);

%fit variogram
[fittedModel, fittedParam, best_RMSE] = kriging.adjust_vario(lags, varioVal);

end