function [A, rcondA] = generateKrigMatrix(X, varioModel, modelParam, trendOrder)
%generateKrigMatrix Creates kriging matrix based on variogram model
%   Creates the kriging matrix by using the coordinates of the points, a
%   handle to a valid variogram model, its parameters and a trend of
%   order<=2

% INPUT :
%   X : the matrix containing the coordinates of all known measurement points
%   varioModel : a fitted valid variogram model
%	modelParam : the parameters of the fitted variogram model
%   trendOrder : the order of the trend function
%       order 0 : f0=1
%       order 1 : f0=1, f1=x, f2=y
%       order 2 : f0=1, f1=x, f2=y, f3=x*x, f4=x*y, f5=y*y

% OUTPUT : 
%	A     : the kriging matrix
%   rcondA: the reciprocal of its conditioning number

% Initialise output matrices
Nmeasures=length(X);
Ntrend=sum(1:trendOrder+1);
A=zeros(Nmeasures+Ntrend, Nmeasures+Ntrend);

% Calculate semivariances for all rows/columns
for j=1:Nmeasures
    A(j,j:Nmeasures)= varioModel(modelParam, sqrt((X(j,1)-X(j:Nmeasures,1)).^2+(X(j,2)-X(j:Nmeasures,2)).^2)); 
end

A(1:Nmeasures, Nmeasures+1)=ones(Nmeasures,1);

if(trendOrder==1)
    % The trend must be included in the matrix
    A(1:Nmeasures,Nmeasures+2)=X(:,1); 
    A(1:Nmeasures,Nmeasures+3)=X(:,2);  
elseif (trendOrder==2)
    A(1:Nmeasures,Nmeasures+2)=X(:,1); 
    A(1:Nmeasures,Nmeasures+3)=X(:,2);  
    A(1:Nmeasures,Nmeasures+4)=X(:,1).*X(:,1); 
    A(1:Nmeasures,Nmeasures+5)=X(:,1).*X(:,2); 
    A(1:Nmeasures,Nmeasures+6)=X(:,2).*X(:,2); 
end
A=A+triu(A)';

% Reciprocal of conditionning number, if close to working precision : high
% risk of singularity.
rcondA=rcond(A);

end

