function [fittedModel,fittedParam,best_RMSE]=variogram(X,Y,lag,range,var)
% Variogram compute the experimental variogram and fit it with a spherical
% model
% INPUT 
% X : position of the sampling point
% Y : measured value
% lag,range : parameters used for experimental variogram computation
% var : estimation of the field variance

% OUTPUT 
% fittedModel : Handle of the model used to fit the variogram
% fittedParam : Parameters of the model

%Compute robust Cressie and Hawkins variogram estimator
[lags, varioVal] = computeRobustVario(range,lag,X,Y);

%fit variogram
[fittedModel, fittedParam, best_RMSE] = adjust_vario(lags,varioVal, var);

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