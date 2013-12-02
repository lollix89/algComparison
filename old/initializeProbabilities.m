function [fieldPrior, fieldPosterior,  mutualInformationMap,  temperatureVector]= initializeProbabilities(lx_,ly_)
            
            fieldPrior=[];
            fieldPosterior= [];
            mutualInformationMap=[];
            temperatureRange=[-12,58];
            temperatureInterval= .2;
            %create temperatureVector------------------
            temperatureVector= (temperatureRange(1):temperatureInterval:temperatureRange(2));
            %initialize probabilities distributions----------
            fieldPrior= initializePriorDistribution(lx_, ly_, temperatureVector);
            fieldPosterior= fieldPrior;
            %initialize mutualInformationMap---------------
            mutualInformationMap= 10.*ones(lx_, ly_);
end

function fieldPrior= initializePriorDistribution(sizeX, sizeY, temperatureVector)

            fieldPrior=[];
            for i=1:sizeX
                for j=1:sizeY
                    fieldPrior(i,j,:)= ones(1,1, size(temperatureVector, 2))./size(temperatureVector, 2);
                end
            end
        end