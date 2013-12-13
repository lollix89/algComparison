function [Path]= greedy(Sx, Sy, fieldX, fieldY, error, allowableDirections, horizon, nWayPoints)
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

Path=[Sx Sy];
currentPosX= Sx;
currentPosY= Sy;
previousDirection= nan;

for i= 1:nWayPoints
    %find the boundaries to interpolate only the part of the error map
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
    [arrivalPoints, tBoundaries]= strategies.avoidSharpBendAndLoop(horizon, previousDirection, allowableDirections, arrivalPoints, tBoundaries, Path);
    
    meanError= strategies.computeMeanError(currentPosX, currentPosY, tBoundaries, error);
    
    [~, idx]= max(meanError);
    %Memorize current direction to forbid nearby opposite directions next iteration
    directionAngles= 0:(360/allowableDirections):359;
    previousDirection= directionAngles(idx);
    
    Path= [Path; arrivalPoints(:,idx)'];
    currentPosX= arrivalPoints(1,idx);
    currentPosY= arrivalPoints(2,idx);
    
end
end




