function [fieldPrior, fieldPosterior,  mutualInformationMap,  temperatureVector]= initializeProbabilities(lx,ly)

temperatureRange=[-12,58];
temperatureInterval= .2;
%create temperatureVector
temperatureVector= (temperatureRange(1):temperatureInterval:temperatureRange(2));
%initialize probabilities distributions----------
fieldPrior= ones(lx*ly, length(temperatureVector))./deal(length(temperatureVector));
fieldPosterior= fieldPrior;
%initialize mutualInformationMap
mutualInformationMap= 10.*ones(ly, lx);
end
