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
[lags, varioVal] = computeRobustVario(range,lag,sCoord,Y);

%fit variogram
[fittedModel, fittedParam, best_RMSE] = adjust_vario(lags, varioVal);

% %Plot fitted variogram
% 
%     figure
%     plot(lags,varioVal, 'ok','MarkerFaceColor','b');hold on
%     h_mod=linspace(0, range,100);
%     plot(h_mod,fittedModel(fittedParam,h_mod),'k-','linewidth',2)
%     set(gca,'FontSize',16)
%     xlabel('distance (m)')
%     ylabel('semi-variogram')
%     hold off

end