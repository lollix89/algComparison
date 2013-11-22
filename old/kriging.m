function [interpValues,krigError]=kriging(X,Y,stdV,meanV,fittedModel,fittedParam,trendOrder,x,y)
% Kriging interpolate the values using kriging interpolation
% input  X                  postion of the sampling points
%        Y                  values of the sampling points
%        stdV/meanV         standard deviation/mean of the data
%        fittedModel        variogram model (usless since we use only 
%                           spherical model here)
%        fittedParam        parameter of the model
%        trendOrder         trend order (0,1,2)
%        x,y                interpolation point positions

% output interpValues       values at interpolation points
%        krigError          kriging error at interpolation point


% Generate kriging matrices
[krigMat,~]=generateKrigMatrix(X, fittedModel, fittedParam, trendOrder);

% if(length(X)>200) % Number of measurements points is high : do NN kriging
%     kdTree = createns(X,'distance','euclidean'); % create kd-tree
%     [interpValuesNorm, krigError]=UniversalKrigingKN(KrigMat, Y, fittedModel, fittedParam, x,y, 15, kdTree, trends);
%     interpValues=interpValuesNorm.*stdV+meanV; % "Denormalize" interpolated values
% else  % Normal kriging
[interpValuesNorm, krigError]=UniversalKriging(krigMat,X,Y, fittedModel, fittedParam, x,y, trendOrder);
interpValues=interpValuesNorm.*stdV+meanV; % "Denormalize" interpolated values
%end

% figure
% hold on
% figure
% [daaa, ch]=contourf(x,y,interpValues',30);hold on;
% plot(X(:,1),X(:,2),'+k')
% set(ch,'edgecolor','none');
% xlabel('X [m]','FontSize',14)
% ylabel('Y [m]','FontSize', 14)
% colorbar
% axis('equal')

% figure
% hold on
% [daaa, ch]=contourf(x,y,krigError',30);hold on;
% plot(X(:,1),X(:,2),'+k')
% set(ch,'edgecolor','none');
% xlabel('Coordinate X [m]','FontSize',14)
% ylabel('Coordinate Y [m]','FontSize', 14)
% colorbar;
% axis('equal')

end