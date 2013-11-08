function [meanDist, vario] = computeRobustVario(range, lag, X, Y)
%computeRobustVario This function computes the Cressie and Hawkins
%variogram estimator

% INPUT : 
%  range : (maximal distance between two points for them to be 
%          considered in the calculation)
%  lag   : distance lag [m], the semivariances corresponding to the 
%          following distance lags intervals will be computed :
%          0,1*lag,2*lag,...,j*lag,...,range
%  X : vector (Nx2) of coordinates (x1,x2) of the sampling points
%  Y : vector (Nx1) of the values at the measurement points

% OUTPUT : 
%  meanDist = Mean distance lags inside each lag interval
%  vario    = median of the semivariances inside each lag interval


nMeasures=length(Y);
nLags=ceil(range/lag);
vario=zeros(nLags,1);
meanDist=zeros(nLags,1);
nMeasuresPerLag=zeros(1,nLags);

% %loop over the pair of measurment points
bin=cell(nLags,1);
for i=1:nMeasures-1
    dist=sqrt((X(i,1)-X(i+1:nMeasures,1)).^2 + (X(i,2)-X(i+1:nMeasures,2)).^2);
    for j=1:nMeasures-i
        if (dist(j)<range)
            indH=ceil(dist(j)/lag);
            bin{indH}= [bin{indH} sqrt(abs(Y(i)-Y(j+i)))];
            meanDist(indH)=meanDist(indH)+dist(j);
            nMeasuresPerLag(indH)=nMeasuresPerLag(indH)+1;
        end
    end
end

%compute Cressie robust vario
for i=1:nLags%loop over the lags
    if(nMeasuresPerLag(i)>0)
       vario(i)=0.5*(median(bin{i})^4)/(0.457+0.494/nMeasuresPerLag(i)+0.045/(nMeasuresPerLag(i)*nMeasuresPerLag(i)));
        meanDist(i)=meanDist(i)/nMeasuresPerLag(i);
    else
       meanDist(i)=nan;
       vario(i)=nan;
    end
end


meanDist(isnan(meanDist))=[];
vario(isnan(vario))=[];


end





% 
% function [meanDist, vario] = computeRobustVario(range, lag, X, Y)
% %computeRobustVario This function computes the robust variogram estimator
% %according to Cressie.
% % INPUT : 
% %  range : (maximal distance between two points for them to be 
% %          considered in the calculation)
% %  lag   : distance lag [m], the semivariances corresponding to the 
% %          following distance lags intervals will be computed :
% %          0,1*lag,2*lag,...,j*lag,...,range
% %  X : vector (Nx2) of coordinates (x1,x2) of the sampling points
% %  Y : vector (Nx1) of the values at the measurement points
% 
% % OUTPUT : 
% %  meanDist = Mean distance lags inside each lag interval
% %  vario    = median of the semivariances inside each lag interval
% 
% 
% nMeasures=length(Y);
% nLags=ceil(range/lag);
% vario=zeros(nLags,1);
% meanDist=zeros(nLags,1);
% nMeasuresPerLag=zeros(nLags,1);
% 
% 
% h=(0:lag:nLags*lag)'; % Vector of distances
% 
% for i=1:nLags%loop over the lags
%     temp=[];
%    for j=1:nMeasures %double loop over the measures
%         for k=j+1:nMeasures
%             a=abs(X(j,1)-X(k,1));
%             if(a<=h(i+1))     
%                b=abs(X(j,2)-X(k,2));
%                if(b<=h(i+1))             
%                    c=sqrt(a*a+b*b);
%                     if(c<=h(i+1)&&c>=h(i))                        
%                         nMeasuresPerLag(i)=nMeasuresPerLag(i)+1;
%                         %vario(i)=vario(i)+sqrt(abs(Y(k)-Y(j)));
%                         temp=[temp, sqrt(abs(Y(k)-Y(j)))];
%                         meanDist(i)=meanDist(i)+c;
%                     end
%               end
%             end
%         end
%     end  
%     % Cressie "robust" estimator
%     if(nMeasuresPerLag(i)>0)
%         %vmean(i)=0.5*((vario(i)/nMeasuresPerLag(i))^4)/(0.457+0.494/nMeasuresPerLag(i)+0.045/(nMeasuresPerLag(i)*nMeasuresPerLag(i)))
%         vario(i)=0.5*(median(temp)^4)/(0.457+0.494/nMeasuresPerLag(i)+0.045/(nMeasuresPerLag(i)*nMeasuresPerLag(i)));
%         meanDist(i)=meanDist(i)/nMeasuresPerLag(i);
%     else
%        meanDist(i)=nan;
%        %vmean(i)=nan;
%        vario(i)=nan;
%     end
% end
% %diff=vmean-vario
% meanDist(isnan(meanDist))=[];
% vario(isnan(vario))=[]
% 
% end
% 
