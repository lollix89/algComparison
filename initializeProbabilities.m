function [fieldPrior, fieldPosterior,  mutualInformationMap,  temperatureVector]= initializeProbabilities(x_,y_)
            
            fieldPrior=[];
            fieldPosterior= [];
            mutualInformationMap=[];
            temperatureRange=[-12,58];
            temperatureInterval= .5;
            %---------------create temperatureVector------------------
            temperatureVector= (temperatureRange(1):temperatureInterval:temperatureRange(2));
            %-------------initialize probabilities distributions----------
            fieldPrior= initializePriorDistribution(size(x_,2), size(y_,2), temperatureVector);
            fieldPosterior= fieldPrior;
            %----------------initialize mutualInformationMap---------------
            mutualInformationMap= 30.*ones(size(x_,2), size(y_,2));
end

function fieldPrior= initializePriorDistribution(sizeX, sizeY, temperatureVector)

            fieldPrior=[];
            for i=1:sizeX
                for j=1:sizeY
                    fieldPrior(i,j,:)= ones(1,1, size(temperatureVector, 2))./size(temperatureVector, 2);
                end
            end
        end