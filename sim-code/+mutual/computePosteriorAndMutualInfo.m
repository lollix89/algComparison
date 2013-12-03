function [prior, posterior, mutualInfo]= computePosteriorAndMutualInfo(prior, posterior, mutualInfo, temperatureV, samples, coords, range, delta, sill, func)

% update the posterior for every cell given the current samples and update
% conditional entropy H(X|Y).

% INPUT
% prior          : grid of the prior 
% posterior      : grid of the posterior
% mutualInfo     : grid of the muutal information
% temperatureV   : the vector of temperatures
% samples        : samples of current iteration
% coords         : coordinates of the smapling points given as x(columns),
%                   y(rows) Nx2
% range          : correlation range of the current field assumed to be
%                   known
% delta          : discretization interval for  

% OUTPUTS
% prior          : Prior map updated
% posterior      : posterior map updated
% mutualInfo     : mutual information map updated


for i=1: size(coords,1)
    [~, closestValueIndex] = min(abs(temperatureV- samples(i)));
    
    for rows= 1:size(prior,1)
        for cols= 1:size(prior,2)
            
            currentDistance= sqrt(sum(([coords(i,2) coords(i,1)]- [(((rows-1)*delta)+delta/2) (((cols-1)*delta)+delta/2)]).^2));
            
            if  isKey(qrs.config,'Function') && strcmp(qrs.config('Function'), 'sph')
                if currentDistance <= range
                    sigma_= .01 + (sill*(1.5*(currentDistance/range)-.5*(currentDistance/range)^3));
                else
                    sigma_= .01+ sill;
                end
            elseif (isKey(qrs.config,'Function') && strcmp(qrs.config('Function'), 'lin')) || ~isKey(qrs.config,'Function')
                sigma_= .01 + currentDistance*(sill/range);
            end
            likelihoodCurrentCell= exp(-0.5 * ((temperatureV - temperatureV(closestValueIndex))./sigma_).^2) ./ (sqrt(2*pi) .* sigma_);
            %likelihoodCurrentCell= normpdf( temperatureV, temperatureV(closestValueIndex), varianceFunction);
            likelihoodCurrentCell= likelihoodCurrentCell./sum(likelihoodCurrentCell);
            %compute posterior current cell
            evidence= sum(likelihoodCurrentCell.*reshape(prior(rows,cols,:), 1, size(prior,3)));
            if evidence ~= 0
                post= (likelihoodCurrentCell.*reshape(prior(rows,cols,:), 1, size(prior,3)))./evidence;
            else
                post= reshape(prior(rows,cols,:), 1, size(prior,3));
            end
            
            posterior(rows,cols,:)= reshape(post, 1,1, size(posterior,3));
            %update mutual information
            %xEntropy= entropy (reshape(prior(x_,y_,:), 1, size(prior,3)));
            RVprobability= reshape(posterior(rows,cols,:), 1, size(posterior,3));
            xyEntropy= sum(-RVprobability(RVprobability> 0).*log2(RVprobability(RVprobability> 0)));
            mutualInfo(rows,cols)= xyEntropy;
            %update prior with computed posterior
            prior(rows,cols,:)= posterior(rows,cols,:);        
        end
    end
    
    
end

end