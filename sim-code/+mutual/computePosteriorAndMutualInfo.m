function [prior, posterior, mutualInfo]= computePosteriorAndMutualInfo(prior, posterior, mutualInfo, temperatureV, samples, coords, range, delta, lx, ly)

% update the posterior for every cell given the current samples and update
% conditional entropy H(X|Y).

% INPUT
% prior          : grid of the prior
% posterior      : grid of the posterior
% mutualInfo     : grid of the mutal information
% temperatureV   : the vector of temperatures
% samples        : samples of current iteration
% coords         : coordinates of the sampling points given as x(columns),
%                   y(rows) Nx2
% range          : correlation range of the current field assumed to be
%                   known
% delta          : discretization interval for probability distributions

% OUTPUTS
% prior          : Prior map updated
% posterior      : posterior map updated
% mutualInfo     : mutual information map updated

sill= qrs.config('Sill');
func= qrs.config('Function');

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
    
    %Choosing the variance function to be used according to the config
    %variable set by arguments
    if strcmp(func, 'spherical')
        sigmas_= (Distances<= range).*(.01 + (sill.*(1.5.*(Distances/range)-.5.*(Distances/range).^3))) + (Distances> range).*(.01+ sill);
    elseif strcmp(func, 'linear')
        sigmas_= .01 + Distances.*(sill/range);
    elseif strcmp(func, 'quadratic')
        sigmas_= .01+ (Distances.^2).*(sill/(range^2));
    elseif strcmp(func, 'cubic')
        sigmas_= .01+ (Distances.^3).*(sill/(range^3));
    end
    
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
    %adding small bias to avid zero evidence case (performance reasons)
    evidence= evidence+ 1e-200;
    evidence= evidence(:,ones(size(likelihood,2),1));
    posterior= (likelihood.*prior)./evidence;
    %posterior= (evidence~= 0).*((likelihood.*prior)./evidence) + (evidence== 0).*(prior);
    
    %Add a small bias to make possible to compute entropy with indexing,otherwise
    %it would take much longer
    posterior= posterior+ 1e-200;
    normalizationFactor= sum(posterior,2);
    posterior= posterior./ normalizationFactor(:, ones(1,size(posterior,2)));
    
    mutualInfoVector= sum(-posterior.*log2(posterior),2);
    mutualInfo= reshape(mutualInfoVector, ly, lx);
    mutualInfo= mutualInfo';
    
    prior= posterior;
    
end

end