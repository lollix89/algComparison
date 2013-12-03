function [tree]= generateACODistanceMatrix(posX, posY, fieldX, fieldY, error, allowableDirections, horizon, nWayPoints)

p0= 0.01;

%Generate all the positions reachable with a path of length nWayPoints
stack= [posX posY];

tree=[];
lowerLimit= max(1, round(stack)- (horizon*(nWayPoints+2)));
upperXLimit= min(fieldX, round(posX)+ (horizon*(nWayPoints+2)));
upperYLimit= min(fieldY, round(posY)+ (horizon*(nWayPoints+2)));
interpolatePatch= error(lowerLimit(2): upperYLimit, lowerLimit(1):upperXLimit);
interpolatePatch= interpolate.interpolate(interpolatePatch);
error(lowerLimit(2): upperYLimit, lowerLimit(1):upperXLimit)= interpolatePatch;

%normalizing the error map
range = max(error(:)) - min(error(:));
error = (error - min(error(:))) ./ range;

futureStack= [];
currentIdx= 2;

for i= 1:nWayPoints +1
    while ~isempty(stack)
        
        tree(end+1).currentNode= stack(1,:);
        [arrivalPoints, tBoundaries]= findAllowableTriangles(stack(1,1), stack(1,2), fieldX, fieldY , allowableDirections, horizon);
        meanError= computeMeanError(stack(1,1), stack(1,2), tBoundaries, error);
        
        arrivalPoints(:, isnan(arrivalPoints(1,:)))= [];
        meanError(isnan(meanError))= [];
        arrivalPoints(3,:)= meanError;
        %Only leave the promising edges, those with a error higher that the
        %mean of all the errors.
        arrivalPoints= arrivalPoints(:, meanError> mean(meanError));
        meanError= arrivalPoints(3,:);
        
        arrivalPoints= arrivalPoints(1:2,:);
        
        tree(end).error= meanError;
        tree(end).pheromone= p0.*ones(1,length(meanError));
        tree(end).nextNode= [arrivalPoints; currentIdx:currentIdx+length(meanError)-1];
        
        currentIdx= currentIdx+length(meanError);
        futureStack= [ futureStack; arrivalPoints'];
        stack(1,:)= [];
    end
    stack= futureStack;
    futureStack= [];
end

end