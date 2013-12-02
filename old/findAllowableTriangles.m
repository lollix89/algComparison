function [arrivalPoints,tBoundaries]= findAllowableTriangles(Sx, Sy, fieldX, fieldY, allowableDirections, horizon)

triangleAngle= 360/allowableDirections;
directionAngles= 0:triangleAngle:359;
triangleAngles= -(triangleAngle/2): triangleAngle: 359;
triangleAngles= kron(triangleAngles,ones(1,2));
triangleAngles= triangleAngles(2:end-1);

arrivalPoints= [Sx + cosd(directionAngles)*horizon; Sy + sind(directionAngles)*horizon] ;

tBoundaries=[];
tBoundaries= [tBoundaries [Sx + cosd(triangleAngles).*((horizon)/cosd((triangleAngle/2))); Sy + sind(triangleAngles).*((horizon)/cosd((triangleAngle/2)))]];

%Check if all boundaries are in range and eventually crop.
for i=1: size(tBoundaries, 2)
    if any(tBoundaries(:,i)< 1)
        tBoundaries(:,i)= max(1, tBoundaries(:,i));
    end
    if tBoundaries(1,i)> fieldX
        tBoundaries(1,i)= min(fieldX, tBoundaries(1,i));
    end
    if tBoundaries(2,i)> fieldY
        tBoundaries(2,i)= min(fieldY, tBoundaries(2,i));
    end
end
deleteIdxArrival=[];
deleteIdxTriangles=[];
for i= 1:size(arrivalPoints,2)
    if any(arrivalPoints(:,i)<1) || arrivalPoints(1,i)> fieldX || arrivalPoints(2,i)> fieldY
        deleteIdxArrival= [deleteIdxArrival i];
        deleteIdxTriangles= [deleteIdxTriangles (i*2)-1 (i*2)];
    end
end
arrivalPoints(:,deleteIdxArrival)= nan;
tBoundaries(:,deleteIdxTriangles)= nan;

end