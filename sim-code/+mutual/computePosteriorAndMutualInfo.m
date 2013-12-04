function [prior, posterior, mutualInfo]= computePosteriorAndMutualInfo(prior, posterior, mutualInfo, temperatureV, samples, coords, range, delta, lx, ly)

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

if isKey(qrs.config,'Sill')
    sill= str2double(qrs.config('Sill'));
else
    sill= 5;
end

if isKey(qrs.config,'Function') && strcmp(qrs.config('Function'), 'sph')
    func= 'spherical';
elseif (isKey(qrs.config,'Function') && strcmp(qrs.config('Function'), 'lin')) || ~isKey(qrs.config,'Function')
    func= 'linear';
end


for i=1: size(coords,1)
    [~, closestValueIndex] = min(abs(temperatureV- samples(i)));
    y = 1:ly;
    Y= y(ones(1,lx),:);
    Y= Y(:);
    x= 1:lx;
    x= x';
    X= x(:, ones(ly,1));
    X= X(:);
    Coord= coords(i*ones(lx*ly,1),:);
    Distances= sqrt(sum((Coord-[((X-1).*delta)+delta/2 ((Y-1)*delta)+delta/2]).^2, 2));
    
    if strcmp(func, 'spherical')
        sigmas_= (Distances<= range).*(.01 + (sill.*(1.5.*(Distances/range)-.5.*(Distances/range).^3))) + (Distances> range).*(.01+ sill);
    elseif strcmp(func, 'linear')
        sigmas_= .01 + Distances.*(sill/range);
    end
    %
    val1 = (temperatureV - temperatureV(closestValueIndex));
    val1= val1(ones(1,length(sigmas_)),:);
    val2= (sqrt(2*pi));
    val2= val2(ones(1,length(sigmas_)))';
    Sigmas_= sigmas_(:,ones(1,size(val1,2)));
    denominator= (val2 .* sigmas_);
    denominator= denominator(:,ones(1,size(val1,2)));
    likelihood= exp(-0.5 .* (val1./Sigmas_).^2) ./ denominator;
    normFactor= sum(likelihood,2);
    normFactor= normFactor(:,ones(1,size(likelihood,2)));
    likelihood= likelihood./normFactor;
    
    evidence= sum((likelihood.*prior), 2);
    evidence= evidence(:,ones(size(likelihood,2),1));
    
    posterior= (evidence~= 0).*((likelihood.*prior)./evidence) + (evidence== 0).*(prior);
    
    mutualInfoVector= zeros(lx*ly, 1);
    for j= 1:size(posterior,1)
        RVProb= posterior(j,:);
        mutualInfoVector(j)= sum(-RVProb(RVProb~= 0).*log2(RVProb(RVProb~= 0)));
    end
    mutualInfo= reshape(mutualInfoVector, ly, lx);
    mutualInfo= mutualInfo';

    prior= posterior;
    
    
end

end