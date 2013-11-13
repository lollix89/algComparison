function [prior, posterior, mutualInfo]= computePosteriorAndMutualInfo(prior, posterior, mutualInfo, temperatureVector, FValues, X, Y, pointX, pointY, range)
sill= 5;
for i=1: size(X,1)
    [~, closestValueIndex] = min(abs(temperatureVector- FValues(i)));
    
    for x_=1:size(pointX,2)
        for y_=1:size(pointY,2)
            
            currentDistance= pdist([X(i) Y(i); pointX(x_) pointY(y_)]);
%             
            if currentDistance <= range
                varianceFunction= .01 + (sill*(1.5*(currentDistance/range)-.5*(currentDistance/range)^3));
            else
                varianceFunction=  .01+ sill+ 2*(sill/range)*(currentDistance-range);
            end
            %varianceFunction= .01 + sill*(currentDistance/range);
            
            likelihoodCurrentCell= pdf('norm', temperatureVector, temperatureVector(closestValueIndex), varianceFunction);
            likelihoodCurrentCell= likelihoodCurrentCell./sum(likelihoodCurrentCell);
            %compute posterior current cell
            evidence= sum(likelihoodCurrentCell.*reshape(prior(x_,y_,:), 1, size(prior,3)));
            if evidence ~= 0
                post= (likelihoodCurrentCell.*reshape(prior(x_,y_,:), 1, size(prior,3)))./evidence;
            else
                post= reshape(prior(x_,y_,:), 1, size(prior,3));
            end
            
            posterior(x_,y_,:)= reshape(post, 1,1, size(posterior,3));
            %update mutual information!!
            %xEntropy= entropy (reshape(prior(x_,y_,:), 1, size(prior,3)));
            xyEntropy= entropy(reshape(posterior(x_,y_,:), 1, size(posterior,3)));
            mutualInfo(x_,y_)= xyEntropy;
            %update prior with computed posterior
            prior(x_,y_,:)= posterior(x_,y_,:);        
        end
    end
    
    
end

end