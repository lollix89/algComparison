function [interpValues, krigError] = UniversalKriging(KrigMatrix, X, Y, varioModel, modelParam, coordX, coordY, trendOrder)
%UniversalKriging Performs universal kriging interpolation of the field using
%a fitted valid variogram model

%  Input :  
%           KrigMatrix   : Kriging Matrix
%           X            : Coordinates of the sampling points
%           Y            : Values at the sampling points
%           varioModel   : a function handle to a valid variogram model
%           modelParam   : the fitted parameters of that model
%           coordX       : A vector containing the x coordinates of the
%                          points where the data shall be interpolated
%           coordY       : A vector containing the y coordinates of the
%                          points where the data shall be interpolated
%           trendOrder   : Order of the polynomial trend
%
%  Output : interpValues : the interpolated values at all coordinates
%           KrigError    : the kriging error at all coordinates

% Universal Kriging
%==========================================================================

% Initialise output matrices
Nx=length(coordX);
Ny=length(coordY);
Nmeasures=length(X);

interpValues=zeros(Nx,Ny);
krigError=interpValues;


[L,U,P]=lu(KrigMatrix);
 
Ntrend=sum(1:trendOrder+1);
for i=1:Ny
    % Create temporary stocking vectors for parallel computing
    B=zeros(Nmeasures+Ntrend,1);
    tempVecI=zeros(1,Nx);%interp value
    tempVecE=zeros(1,Nx);%error
    y=coordY(i);
    
    for j=1:Nx
    x=coordX(j);
    h=sqrt((X(:,1)-x).^2+(X(:,2)-y).^2);
    

    B(1:Nmeasures)=varioFun(modelParam,h);    
    
    %(h<param(2)).*(param(1)*(3.*h./(2*param(2))-1/2*(h./param(2)).^3))+(h>=param(2)).*(param(1));
    
    B(Nmeasures+1)=1;
    if(trendOrder==1)
        B(Nmeasures+2)=x;
        B(Nmeasures+3)=y;
    elseif(trendOrder==2)
        B(Nmeasures+2)=x;
        B(Nmeasures+3)=y;
        B(Nmeasures+4)=x*x;
        B(Nmeasures+5)=x*y;
        B(Nmeasures+6)=y*y;
    end

    u=L\(P*B);
    lambdaMu=U\u;
    
    % Parallel computing makes the use of a temporary stocking vector
    % necessary
    tempVecI(j)=Y'*lambdaMu(1:Nmeasures);
%     B(8:9)=B(8:9)+0.5;
%     B(14:15)=B(14:15)+0.5;
    
    tempVecE(j)=B'*lambdaMu;

    end
    interpValues(i,:)=tempVecI;
    krigError(i,:)=tempVecE;
end

end

function val=varioFun(param,h)
a=h/param(2);
val=((h<param(2))*0.5.*(3*a-a.^3) + (h>=param(2)))*(param(1));
end

