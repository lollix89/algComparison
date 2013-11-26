function [arrivalPoint]= greedy(Sx, Sy, fieldX, fieldY, velocity, error, allowableDirections)
% optimize the path of the robot in function of the error
%       input S                   starting postion of the robot
%       velocity                  velocity of the robot (assumed constant)
%       error                     distance matrix
%       allowableDirections       number of possible directions for the
%       robot
%output bestDirection             best direction found

computationTime= 3; % seconds



tBoundaries= findAllowableTriangles(Sx,Sy, fieldX, fieldY ,velocity, allowableDirections, computationTime);

meanError= computeMeanError(Sx,Sy, tBoundaries, error);

[~, idx]= max(meanError);
arrivalPoint= mean([tBoundaries(:,(idx*2)) tBoundaries(:,(idx*2)-1)], 2);
arrivalPoint= arrivalPoint';

end

function [tBoundaries]= findAllowableTriangles(Sx, Sy, fieldX, fieldY, velocity, allowableDirections, computationTime)


triangleArea= 360/allowableDirections;
directionAngles= 0:triangleArea:359;


tBoundaryX=[];
tBoundaryY=[];

triangleBoundaries= [directionAngles- (triangleArea/2);  directionAngles+ (triangleArea/2)];
triangleBoundaries= reshape(triangleBoundaries, 1, allowableDirections*2);


 for i=1:size(triangleBoundaries,2)
     %check boundaries on x axis
     if cosd(triangleBoundaries(i)) < 0 && Sx > 1 
         tBoundaryX= [tBoundaryX  min(Sx-1, (Sx + cosd(triangleBoundaries(i))*(velocity*computationTime)))];
         
     elseif cosd(triangleBoundaries(i)) > 0 && Sx < fieldX
         tBoundaryX=  [tBoundaryX  min(fieldX- Sx,  Sx+ cosd(triangleBoundaries(i))*(velocity*computationTime))];
         
     elseif (Sx==1 && cosd(triangleBoundaries(i)) < 0) || (Sx==  fieldX && cosd(triangleBoundaries(i)) > 0)
         tBoundaryX=  [tBoundaryX Sx+ cosd(triangleBoundaries(i))*(velocity*computationTime)];
     end
    %check boundaries on y axis
    if sind(triangleBoundaries(i)) < 0 && Sy > 1 
        tBoundaryY= [tBoundaryY min(Sy-1, (Sy + sind(triangleBoundaries(i))*(velocity*computationTime)))];
        
    elseif sind(triangleBoundaries(i)) >= 0 && Sy < fieldY  
        tBoundaryY=  [tBoundaryY min( fieldY- Sy , Sy+ sind(triangleBoundaries(i))*(velocity*computationTime))];
        
    elseif (Sy==1 && sind(triangleBoundaries(i))< 0 )|| (Sy == fieldY && sind(triangleBoundaries(i))> 0) 
        tBoundaryY= [tBoundaryY Sy+ sind(triangleBoundaries(i))*(velocity*computationTime)];
    end
 end
 deleteIdx=[];
 for j=1:size(triangleBoundaries,2)
     if  (tBoundaryX(1,j)> fieldX || tBoundaryX(1,j)< 1 ...
             || tBoundaryY(1,j)> fieldY || tBoundaryY(1,j)< 1 ) 
         
         if (~isempty(deleteIdx) && deleteIdx(end)~= j-1 && deleteIdx(end)~= -1) || (isempty(deleteIdx) && j~=1 )
             tBoundaryX(1,j)= Sx+ (round(cosd(triangleBoundaries(j)))*(velocity*computationTime));
             tBoundaryY(1,j)= Sy+ (round(sind(triangleBoundaries(j)))*(velocity*computationTime));
             deleteIdx= [deleteIdx -1];
         else
             deleteIdx= [deleteIdx j];
         end
     elseif ~isempty(deleteIdx) && deleteIdx(end)== j-1
         deleteIdx=deleteIdx(1:end-1);
         tBoundaryX(1,j-1)= Sx+ (round(cosd(triangleBoundaries(j-1)))*(velocity*computationTime));
         tBoundaryY(1,j-1)= Sy+ (round(sind(triangleBoundaries(j-1)))*(velocity*computationTime));
     end
 end
 
deleteIdx = deleteIdx(deleteIdx~=-1);
tBoundaryX(deleteIdx)= [];
tBoundaryY(deleteIdx)= [];
tBoundaries = [round(tBoundaryX); round(tBoundaryY)];

end

%for each triangle find the point lying in it and compute the mean error
function [meanError]= computeMeanError(Sx, Sy, tBoundaries, error)

meanError=[];

for i=1:2:size(tBoundaries, 2)
    minX= min([Sx tBoundaries(1,i) tBoundaries(1,i+1)]);
    maxX= max([Sx tBoundaries(1,i) tBoundaries(1,i+1)]);
    minY= min([Sy tBoundaries(2,i) tBoundaries(2,i+1)]);
    maxY= max([Sy tBoundaries(2,i) tBoundaries(2,i+1)]);
    
    innerPoints=[];
    width= maxX- minX;
    heigth= maxY- minY;
    
    for j=1: width*heigth
        currentPnt= [ mod(j-1, width)+ minX; floor((j-1)/width)+minY; 0];
        tBoundaryA= [tBoundaries(:,i); 0];
        tBoundaryB= [tBoundaries(:,i+1); 0];
        tBoundaryC= [Sx; Sy; 0];
        %check if current point is inside the triangle
        ab= cross(currentPnt-tBoundaryA, tBoundaryB - tBoundaryA);
        bc= cross(currentPnt-tBoundaryB, tBoundaryC- tBoundaryB);
        ca= cross(currentPnt-tBoundaryC, tBoundaryA- tBoundaryC);
        
        if sign(ab(3))*sign(bc(3))>0 && sign(bc(3))*sign(ca(3))> 0           
            innerPoints= [innerPoints error(currentPnt(2), currentPnt(1))];
        end
    end
    meanError= [meanError mean(innerPoints)];
    
end


end



