function [bestpath]= greedy(x,y,S,Dh,Dv,Ddu,Ddd, pathLength)
% optimize the path of the robot in function of the kriging error using ACO
% input x,y                 positions of the allowable waypoints
%       S                   starting postion of the robot
%       Dh,Dv,Ddu,Ddd       distance matrices (horizontal,vertical,growing
%                           diagonal and descending diagonal edes)
%output bestPath            best path found

lx=length(x);ly=length(y);

bestPath=S;
currentNode=S;

for i=1:pathLength-1
    neighbourNodes= getNeighbourNodes(currentNode,bestPath,lx,ly);
    meanError= getDistance(currentNode,neighbourNodes,Dh,Dv,Ddu,Ddd)';
    [~, bestNeighbourIdx]= max(meanError);
    if isempty(neighbourNodes)
        disp('stopped in debugger')
    end
    currentNode = neighbourNodes(bestNeighbourIdx);
    bestPath=[bestPath currentNode];
end

indY = ceil(bestPath/lx);
indX = bestPath -(indY-1)*lx;
bestpath(:,1)= x(indX);
bestpath(:,2)= y(indY);

end

function neighbourNodes=getNeighbourNodes(k,visitedNodes,lx,ly)
indY = ceil(k/lx);
indX = k -(indY-1)*lx;

if(indX>1 && indX<lx && indY>1 && indY<ly)
    %neighbourNodes =[k-1, k+1, k+lx, k-lx];
    neighbourNodes =[k-1, k+1, k+lx, k-lx, k+lx-1, k+lx+1, k-lx+1, k-lx+1];
else
    if(indX==1)
        if(indY==1)
            neighbourNodes=[k+1, k+lx,k+lx+1];
        elseif (indY==ly)
            neighbourNodes=[k+1, k-lx,k-lx+1];
        else
            neighbourNodes=[k+1,k-lx, k+lx, k-lx+1, k+lx+1];
        end
    elseif(indX==lx)
        if(indY==1)
            neighbourNodes=[k-1, k+lx, k+lx-1];
        elseif (indY==ly)
            neighbourNodes=[k-1, k-lx, k-lx-1];
        else
            neighbourNodes=[k-1, k-lx,k+lx, k-lx-1, k+lx-1];
        end
    elseif(indY==1)
        neighbourNodes=[k-1, k+1, k+lx, k+lx-1, k+lx+1];
    else
        neighbourNodes=[k-1, k+1, k-lx, k-lx-1, k-lx+1];
    end
end

%directional conditions
if length(visitedNodes)>1
    k0=visitedNodes(end-1);
    if abs(k-k0)==1%horizontal
        visitedNodes=[visitedNodes k0+lx k0-lx];
    elseif abs(k-k0)==lx%vertical
        visitedNodes=[visitedNodes k0+1 k0-1];
    elseif abs(k-k0)==lx+1%diag up
        visitedNodes=[visitedNodes k0+lx*sign(k-k0) k0+1*sign(k-k0)];
    elseif abs(k-k0)==lx-1%diag down
        visitedNodes=[visitedNodes k0+lx*sign(k-k0) k0-1*sign(k-k0)];
    end
end

deleteList=[];
for i=1:length(neighbourNodes)
   if any(visitedNodes==neighbourNodes(i))
        deleteList=[deleteList, i];
   end
end

if length(deleteList)< length(neighbourNodes)
    neighbourNodes(deleteList)=[];
end

end

function dist=getDistance(current,neighbours,Dh,Dv,Ddu,Ddd)
[~,lx]=size(Dv);
dist=zeros(1,length(neighbours));

for i=1:length(neighbours)
    k=min([current,neighbours(i)]);
    indY = ceil(k/lx);
    indX = k -(indY-1)*lx;
    if abs(current-neighbours(i))==1 
        dist(i)=Dh(indY,indX);
    elseif abs(current-neighbours(i))==lx
        dist(i)=Dv(indY,indX);
    elseif abs(current-neighbours(i))==lx+1
        dist(i)=Ddu(indY,indX);
    elseif abs(current-neighbours(i))==lx-1
        dist(i)=Ddd(indY,indX-1);
    end
end
end