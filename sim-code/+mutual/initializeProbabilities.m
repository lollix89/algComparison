function [fieldPrior, fieldPosterior,  mutualInformationMap,  temperatureVector]= initializeProbabilities(lx,ly)
%This function initialize all the probabilities distributions required by
%the mutual information approach, in particular prior distribution is
%initialized as a uniform distribution and mutualInformation as a constant
%map.

%input: lx,ly dimensions of the probability sitributions grid, ie the
%number of distribution probabilities

%output: fieldPrior         matrix of prior distributions
%        fieldPosterior     matrix of posterior distributions
%  mutualInformationMap     matrix of constant mutual information distribution
%  temperatureVector        vector of temperature vector

temperatureRange=[-12,58];
temperatureInterval= .2;
temperatureVector= (temperatureRange(1):temperatureInterval:temperatureRange(2));
fieldPrior= ones(lx*ly, length(temperatureVector))./deal(length(temperatureVector));
fieldPosterior= fieldPrior;
mutualInformationMap= 10.*ones(ly, lx);
end
