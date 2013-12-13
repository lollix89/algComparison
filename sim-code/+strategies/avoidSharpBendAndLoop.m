function [arrivalPoints, tBoundaries]= avoidSharpBendAndLoop(horizon, previousDirection, allowableDirections, arrivalPoints, tBoundaries, path)

directionAngles= 0:(360/allowableDirections):359;

%Find forbidden directions for current iteration and delete
%corresponding values and mean error. The forbidden directions are used
%to avoid the robot to turn back on the path it came from.
if ~isnan(previousDirection)
    radiusForbiddenDirections= floor(allowableDirections/5);
    [~, closestIdx] = min(abs(directionAngles- mod(previousDirection + 180,360)));
    forbiddenDirs= (mod(directionAngles(closestIdx)+ (-radiusForbiddenDirections*...
        (360/allowableDirections):(360/allowableDirections):radiusForbiddenDirections*(360/allowableDirections)), 360));
    [~, idx1, ~]= intersect(directionAngles, forbiddenDirs);
    if sum(~isnan(arrivalPoints(1,:)))> length(idx1)
        idx1= idx1(:)';
        arrivalPoints(:,idx1)= nan;
        idx2= idx1.*2;
        idx2=idx2(ones(1,2),:);
        idx2=idx2(:);
        idx2(1:2:end)= idx2(1:2:end)-1;
        tBoundaries(:,idx2)= nan;
    end 
end
%Delete those points that fall within horizon from the previous points in the path of
%current iteration (try to avoid sharp bends or closed paths)
if ~isempty(path)
    idxP = 1:size(path,1);
    IdxP= idxP(ones(1,size(arrivalPoints,2)),:);
    IdxP= IdxP(:);
    tmpHistory= path(IdxP,:);
    
    idxA= 1:size(arrivalPoints,2);
    idxA= idxA';
    IdxA= idxA(:, ones(size(path,1),1));
    IdxA= IdxA(:);
    tmpArrivalPoints= arrivalPoints(:, IdxA)';
    
    Distances= sqrt(sum((tmpHistory-tmpArrivalPoints).^2, 2));
    deleteIdx= unique(mod(find(Distances < (horizon-1e-8))-1, size(arrivalPoints,2))+1);
    if ~isempty(deleteIdx) && sum(~isnan(arrivalPoints(1,:)))> length(deleteIdx)
        deleteIdx=deleteIdx(:);
        arrivalPoints(:,deleteIdx)= nan;
        deleteIdx2= deleteIdx.*2;
        deleteIdx2=deleteIdx2(:,ones(1,2));
        deleteIdx2= deleteIdx2';
        deleteIdx2=deleteIdx2(:);
        deleteIdx2(1:2:end)= deleteIdx2(1:2:end)-1;
        tBoundaries(:,deleteIdx2)= nan;
    end
end
end