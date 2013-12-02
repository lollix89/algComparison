function [path]= greedy(Sx, Sy, fieldX, fieldY, error, allowableDirections, horizon, nWayPoints)
% optimize the path of the robot in function of the error
%       input S                   starting postion of the robot
%       error                     distance matrix
%       allowableDirections       number of possible directions for the
%       robot
%output path             best path found (Nx2)

path=[];
currentPosX= Sx;
currentPosY= Sy;
previousDirection= nan;

for i= 1:nWayPoints
    
    extremeLeft= max(1, round(currentPosX-2*horizon));
    extremeRight= min(fieldX, round(currentPosX+ 2*horizon));
    extremeUp= min(fieldY, round(currentPosY+ 2*horizon));
    extremeDown= max(1, round(currentPosY- 2*horizon));
    interpolatePatch= error(extremeDown:extremeUp, extremeLeft:extremeRight);
    error(extremeDown:extremeUp, extremeLeft:extremeRight)= interpolate(interpolatePatch);
    
    %normalizing the error map
    range = max(error(:)) - min(error(:));
    error = (error - min(error(:))) ./ range;
    
    [arrivalPoints, tBoundaries]= findAllowableTriangles(currentPosX, currentPosY, fieldX, fieldY , allowableDirections, horizon);
    
    meanError= computeMeanError(currentPosX, currentPosY, tBoundaries, error);
    
    %Find forbidden directions for current iteration and delete
    %corresponding values and mean error.
    if ~isnan(previousDirection)
        radiusForbiddenDirections= 1;
        [~, closestIdx] = min(abs(directionAngles- mod(previousDirection + 180,360)));
        forbiddenDirs= (mod(directionAngles(closestIdx)+ (-radiusForbiddenDirections*...
            (360/allowableDirections):(360/allowableDirections):radiusForbiddenDirections*(360/allowableDirections)), 360));
        [~, idx1, ~]= intersect(directionAngles, forbiddenDirs);
        if sum(~isnan(arrivalPoints(1,:)))> length(idx1)
            
            arrivalPoints(3,:)= meanError;
            arrivalPoints(:,idx1)= nan;
            meanError= arrivalPoints(3,:);
            arrivalPoints= arrivalPoints(1:2,:);
        end
    end
    
    [~, idx]= max(meanError);
    %Memorize current direction to forbid nearby opposite directions next iteration
    directionAngles= 0:(360/allowableDirections):359;
    previousDirection= directionAngles(idx);
    path= [path; arrivalPoints(:,idx)'];
    
    currentPosX= arrivalPoints(1,idx);
    currentPosY= arrivalPoints(2,idx);
    
end
path= [Sx Sy; path ];


end




