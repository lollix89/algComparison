function [path]= greedy(Sx, Sy, fieldX, fieldY, error, allowableDirections, horizon, nWayPoints)
% This function computes the path of the robot as function of the error of
% current path thus using a greedy strategy

%input 
%       Sx,Sy:                   starting postion of the robot
%       fieldX,fieldY:           Extension of the field
%       error:                   error map
%       allowableDirections:     number of possible directions for the
%       horizon:                 lookahead distance of the robot to compute
%                                the mean error
%       nWayPoints:              Length of the path to be computed

%output
%       path:                    best path found of length nWayPoints (Nx2)

path=[];
currentPosX= Sx;
currentPosY= Sy;
previousDirection= nan;

for i= 1:nWayPoints
    %find the boundaries to cinterpolate only the part of the error map
    %needed and save computational time
    extremeLeft= max(1, round(currentPosX-2*horizon));
    extremeRight= min(fieldX, round(currentPosX+ 2*horizon));
    extremeUp= min(fieldY, round(currentPosY+ 2*horizon));
    extremeDown= max(1, round(currentPosY- 2*horizon));
    interpolatePatch= error(extremeDown:extremeUp, extremeLeft:extremeRight);
    error(extremeDown:extremeUp, extremeLeft:extremeRight)= interpolate.interpolate(interpolatePatch);  
    %normalizing the error map
    range = max(error(:)) - min(error(:));
    error = (error - min(error(:))) ./ range;
    
    [arrivalPoints, tBoundaries]= strategies.findAllowableTriangles(currentPosX, currentPosY, fieldX, fieldY , allowableDirections, horizon);
    meanError= strategies.computeMeanError(currentPosX, currentPosY, tBoundaries, error);
    
    %Find forbidden directions for current iteration and delete
    %corresponding values and mean error. The forbidden directions are used
    %to avoid the robot to turn back on the path it came from.
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




