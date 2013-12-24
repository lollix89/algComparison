function [nodes,nextNodeIdxs,errors,pheromones]= generateACODistanceMatrix(posX, posY, fieldX, fieldY, error, allowableDirections, horizon, nWayPoints)
%This function generates on the fly all the points reachable from the
%current position with a path of length nWayPoints, and returns a structure
%of the tree used from findACO to compute the best path.

%Input:
%       posX, posY:             Current position of the robot
%       fieldX, fieldY:         Dimensions of the field
%       error:                  Error map
%       allowableDirections:    Number of directions allowed from a current
%                               point (nb increasing this values increases the commputational time since the number of reachable nodes increases)
%       horizon:                Horizon of the robot, ie the length of the points considered to compute the mean error
%       nWayPoints:             Length of the path


p0= 0.01;       %Initial quantity of pheromone
stack= [posX; posY];
nodes=[];
errors= [];
futureStack= [];
previousDirection= nan;

%Find the boundaries to interpolate the error map only where needed to make the interpolation
%faster
lowerLimit= max(1, round(stack)- (horizon*(nWayPoints+2)));
upperXLimit= min(fieldX, round(posX)+ (horizon*(nWayPoints+2)));
upperYLimit= min(fieldY, round(posY)+ (horizon*(nWayPoints+2)));
interpolatePatch= error(lowerLimit(2): upperYLimit, lowerLimit(1):upperXLimit);
interpolatePatch= interpolate.interpolate(interpolatePatch);
error(lowerLimit(2): upperYLimit, lowerLimit(1):upperXLimit)= interpolatePatch;
%normalize the error map
range = max(error(:)) - min(error(:));
error = (error - min(error(:))) ./ range;

for i= 1:nWayPoints +1
    while ~isempty(stack)   
        [arrivalPoints, tBoundaries]= strategies.findAllowableTriangles(stack(1,1), stack(2,1), fieldX, fieldY , allowableDirections, horizon);
        if ~isempty(nodes);
            [arrivalPoints, tBoundaries]= strategies.avoidSharpBendAndLoop(horizon, previousDirection, allowableDirections, arrivalPoints, tBoundaries, nodes(1:2,:)');
        end
        meanError= strategies.computeMeanError(stack(1,1), stack(2,1), tBoundaries, error);
        
        if sum(isnan(meanError))~= sum(isnan(tBoundaries(1,:)))/2
            disp('horizon is too short');   
        end
        
        arrivalPoints(:, isnan(arrivalPoints(1,:)))= [];
        meanError(isnan(meanError))= [];

        arrivalPoints(3,:)= meanError;
        %Only leave the promising edges, those with an error higher that the
        %mean of all the errors in order to save computational time.
        arrivalPoints= arrivalPoints(:, meanError >= mean(meanError));    
        meanError= arrivalPoints(3,:);
        arrivalPoints= arrivalPoints(1:2,:);
        nodes(:,end+1)= [stack(:,1); length(meanError)];
        errors=[errors meanError];
        futureStack= [ futureStack arrivalPoints];
        stack(:,1)= [];
    end
    stack= futureStack;
    futureStack= [];
end
nextNodeIdxs= 2:length(errors);
pheromones= p0.*ones(1,length(nextNodeIdxs));

end