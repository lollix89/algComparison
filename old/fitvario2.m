function [param,RMSE] = fitvario2(h, vario, param0, fct, varargin)
% Fitvario : fits a variogram model to a set of data using the
% Levenberg-Marcquart algorithm

% INPUT :    h : vector of distances h
%            vario : vector of empirical variogram values for the distances h 
%            param0 : initial values vector for the parameters of the model
%            model : function handle of the variogram model
%            Optional : Weighting matrix W and parameter "FORCE"
% OUTPUT : 
%		     param : fitted parameters of the variogram model
%            RMSE : Root mean square error


% Variable declarations
%==========================================================================

param=zeros(length(param0),1); % Vector of the fitted parameters
J=zeros(length(vario),length(param)); % Jacobian matrix
tau=0.1;  % initial step parameter in the L-M algorithm

var_est=fct(param0,h); % Estimated variogram

res0=vario-var_est;  %Initial residuals
res1=10; % New residuals, high value just to start the iterations
dh=0.001; % dh in the derivation calculations
i=1;
param=param0;


% Assign matrix of weights
if(~isempty(varargin)&&isnumeric(varargin{1}))
    W=varargin{1};
else
    W=diag(ones(length(vario),1));
end
   


var_est=fct(param,h);
bestRMSE=sqrt(sum((var_est-vario).^2.*diag(W)));
bestParam=param;
% Iterations of the L-M algorithm
%==========================================================================
while (abs(sum(res1))>10^-3&&(i<2)) % Check residuals
	
    % Calculation of the Jacobian matrix by finite differences
    a=fct(param,h);
    for j=1:length(param)
        param(j)=param(j)+dh;
        b=fct(param,h);
        param(j)=param(j)-dh;
        J(:,j)=(b-a)/dh;
    end
    
    H=J'*W*J;
    M1=(H+tau*diag(diag(H)));
    M2=J'*W;

    dh_p=M1\M2*res0;
    param=param+dh_p';

    % Avoid imaginary numbers
    param=abs(param);
    var_est=fct(param,h);

    res1=vario-var_est;

    % Adjust damping factor tau
    if(sum(abs(res1))<sum(abs(res0)))
        tau=10*tau;
    else
        tau=tau/10;
    end

    i=i+1;
    res0=res1;
    RMSE=sqrt(sum((var_est-vario).^2.*diag(W)));
    if(RMSE<bestRMSE)
       bestRMSE=RMSE;
       bestParam=param;
    end
end

param=bestParam;

end

