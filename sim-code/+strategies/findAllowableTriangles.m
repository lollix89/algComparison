function [arrivalPoints,tBoundaries]= findAllowableTriangles(Sx, Sy, fieldX, fieldY, allowableDirections, horizon)
%This function computes the arrival points from the current robot position
%considering the number of possible directions. The function computes also
%the boundaries of all the triangles used to compute the mean error.

%Input: 
%       Sx,Sy:                  Current position of the robot
%       fieldX, fieldY:         Extension of the field
%       allowableDirections:    Number of directions allowed from the
%                               current position

%output:
%       arrivalPoints:          Coordinates of arrival points already
%                               cropped to respect the boundaries. The
%                               arrival points not respecting the
%                               boundaries are returned as nan.
%       tBoundaries:            Boundaries of the triangles for each
%                               allowed directions. The number of tBoundaries is always the double
%                               of the arrivalPoints, since every direction
%                               has two boundaries. If an arrival point is
%                               nan the two respective boundaries are nan


triangleAngle= 360/allowableDirections;
directionAngles= 0:triangleAngle:359;
triangleAngles= -(triangleAngle/2): triangleAngle: 359;
triangleAngles= triangleAngles(ones(1,2),:);
triangleAngles= triangleAngles(:)';

triangleAngles= triangleAngles(2:end-1);

arrivalPoints= [Sx + cosd(directionAngles)*horizon; Sy + sind(directionAngles)*horizon] ;

tBoundaries= [Sx + cosd(triangleAngles).*((horizon)/cosd((triangleAngle/2))); Sy + sind(triangleAngles).*((horizon)/cosd((triangleAngle/2)))];

%Check if all boundaries are in range and eventually crop on the direction.
%Handle boundaries
if Sx==1 || Sx== fieldX || Sy==1 || Sy==fieldY
    tBoundaries= max(1, tBoundaries);
    tBoundaries(1,:)= min(fieldX, tBoundaries(1,:));
    tBoundaries(2,:)= min(fieldY, tBoundaries(2,:));
%Handle crossing with boundaries
else
    xBox= [0 0 fieldX fieldX 0];
    yBox= [0 fieldY fieldY 0 0];
    
    segmentsX= [tBoundaries(1,:); ones(2,size(tBoundaries,2)).*Sx];
    segmentsY= [tBoundaries(2,:); ones(2,size(tBoundaries,2)).*Sy];
    segmentsX= segmentsX(:)';
    segmentsY= segmentsY(:)';
    [xi,yi,idx] = polyxpoly(segmentsX, segmentsY, xBox, yBox);
    tBoundaries(:,ceil(idx(:,1)/3))= [xi yi]';
    
    segmentsX= [arrivalPoints(1,:); ones(2,size(arrivalPoints,2)).*Sx];
    segmentsY= [arrivalPoints(2,:); ones(2,size(arrivalPoints,2)).*Sy];
    segmentsX= segmentsX(:)';
    segmentsY= segmentsY(:)';
    [xi,yi,idx] = polyxpoly(segmentsX, segmentsY, xBox, yBox);
    arrivalPoints(:,ceil(idx(:,1)/3))= [xi yi]';
    
end
%Refine arrival points
deleteIdxArrival=[];
deleteIdxTriangles=[];
for i= 1:size(arrivalPoints,2)
    if pdist([Sx Sy; arrivalPoints(:,i)'])< horizon/2 || pdist([Sx Sy; arrivalPoints(:,i)'])< 5 || any(arrivalPoints(:,i)<1) || arrivalPoints(1,i)> fieldX || arrivalPoints(2,i)> fieldY
        deleteIdxArrival= [deleteIdxArrival i];
        deleteIdxTriangles= [deleteIdxTriangles (i*2)-1 (i*2)];
    end
end
arrivalPoints(:,deleteIdxArrival)= nan;
tBoundaries(:,deleteIdxTriangles)= nan;
end