function [best_model, best_param, best_RMSE] = adjust_vario(h, vario, var)
%ADJUST_VARIO Key function of the variogram fitting
%   This function declares all valid variogram models, determines
%   approximations of initial parameters and tries all different variogram
%   models by calling the fit_vario function. The best model is then kept.

% INPUT : h : distance lags of the semivariance estimations
%         vario : semivariances values
%         var : an estimation of the variance of the process (to compute
%         an estimation of the sill)
% OUTPUT : best_model : the best model that was found
%          best_param : the fitted parameters for that given model
%          best_RMSE  : the RMSE of the best model fitting

% Approximation of the initial model parameters
%==================================================================================================
nugget=0;

%inital value of the sill
%sill = var;%valeur assymptotique théoriquement égale à la variance
sill=(max(vario)+mean(vario))/2;
%sill=var;
%sill = polyval(polyfit(h,vario,1),120);
%initial value of the range
range = h((vario>0.9*sill));
if(~isempty(range))
    range = range(1);
else
    range=h((vario>0.9*max(vario))); % If we don't reach the estimated sill, juste take 90% of max
    range=range(1);
end

% 
% % Parameters for other models
% damp_hole = 2*range; % distance lag of the damping hole for the dampened hole model
% if(isempty(damp_hole))
%     damp_hole=0;
% end
% % Estimations for the power model
% index_big_nugget=find(vario>nugget); % Avoid to get negative number in log calculation
% if(isempty(index_big_nugget))
%     exponent_pow=1;
% else
%     exponent_pow=(log(vario(length(vario))-nugget)-log(vario(index_big_nugget(1))-nugget))/(log(h(length(vario)))-log(h(index_big_nugget(1))));
% end
% % Correct estimations of the exponent to ensure negative-definitness 
% if(exponent_pow>=2)
%     exponent_pow=2;
% elseif(exponent_pow<0)
%     exponent_pow=0;
% end

% Estimation of the constant C (y=C*x^a)
%const_pow=(vario(length(vario))-nugget)/h(length(vario))^exponent_pow;
%==================================================================================================
% Definition of the variogram models
%==================================================================================================
sph=@(param,h) ((h<param(2))*0.5.*(3*h/(param(2))-(h/param(2)).^3) + (h>=param(2)))*(param(1));
%sph=@(param,h) param(1)+(h<param(3)).*((param(2)-param(1))*(3.*h./(2*param(3))-1/2*(h./param(3)).^3))+(h>=param(3)).*(param(2)-param(1));
% expo=@(param,h) (param(2)-param(1))*(1-exp(-h/param(3)))+param(1);
% gauss=@(param,h) (param(2)-param(1))*(1-exp(-(h/param(3)).^2))+param(1);
% damp=@(param,h) (param(2)-param(1))*(1-exp(-3*h/param(4)).*cos((h/param(3))*pi))+param(1);
% pow=@(param,h) param(1)+param(2)*h.^(param(3));
% piecewise_lin_vario=@(param,h) (h<=param(5)).*(param(2)+param(1)*h)+(h>param(5)).*(param(4)+param(3)*(h-param(5)));
%==================================================================================================

models_handles{1}=sph;
% models_handles{2}=expo;
% models_handles{3}=damp;
% models_handles{4}=pow;
%models_handles{7}=piecewise_lin_vario; % Was not a bad idea but it is too
% discontinuous...
best_RMSE=inf; % Just to start
W=diag(logspace(3,1,length(vario)),0); % Give more weights to the points at
% small distance lags since they have more weight in the interpolation

%==========================================================================
for i=1:1%length(models_handles)
    % Assign according intitial parameters estimations to the models
    if(isequal(models_handles{i},sph))
        param0=[nugget,sill,range];
        param0=[sill,range];
       [fitted_param, RMSE, ]=fitvario2(h, vario, param0, models_handles{i},W);
       %RMSE=1;
       %fitted_param=param0;
    elseif(isequal(models_handles{i},expo))
        param0=[nugget,sill,range];
        [fitted_param, RMSE]=fitvario(h, vario, param0, models_handles{i},W);
    elseif(isequal(models_handles{i},damp))
        param0=[nugget, sill, range, damp_hole];
        [fitted_param, RMSE]=fitvario(h, vario, param0, models_handles{i},W);
    elseif(isequal(models_handles{i},pow))
        param0=[nugget, const_pow, exponent_pow];
        [fitted_param, RMSE]=fitvario(h, vario, param0, models_handles{i},W);  
    end
   % Find out which model is the best
   if(RMSE<best_RMSE)
       best_RMSE=RMSE;
       best_model=models_handles{i};
       best_param=fitted_param;
   end 
end%=======================================================================

% Check if variogram model is an increasing function
% otherwise something is wrong, in that cas adjust a simple linear
% model instead after removing outliers.
% if(best_model(best_param, h(1))>best_model(best_param,h(end)))
%   best_model=pow;
%   best_param=[0, (max(vario)-min(vario))/h(end), 1];
% end

end
