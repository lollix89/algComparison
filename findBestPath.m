function [bestPath,bestFitness]=findBestPath(x,y,S,Dh,Dv,Ddu,Ddd,maxIter,alpha,beta)
% optimize the path of the robot in function of the kriging error using ACO
% input x,y                 positions of the allowable waypoints
%       S                   starting postion of the robot
%       Dh,Dv,Ddu,Ddd       distance matrices (horizontal,vertical,growing
%                           diagonal and descending diagonal edes)
%       maxIter             Maximum number of iteration
%       alpha,beta          ACO parameters
%output bestPath            best path found
%       bestFiteness        fitness of the best path

lx=length(x);ly=length(y);

m=15; %number of ants.
%alpha=1;
%beta=3;%greedy
% alpha=0.8;
% beta=4;
p0=0.01; %initial pheromone concentration

mu=0.01;
rho=0.05;



tauH=ones(ly,lx-1)*p0;
tauV=ones(ly-1,lx)*p0;
tauDu=ones(ly-1,lx-1)*p0;
tauDd=tauDu;

bestFitness=0;

%% loop
for K=1:50
    %initialization--------------------------------------------------------
    visitedNodes=cell(m,1);
    currentNode=zeros(m,1);
    for I=1:m
       visitedNodes{I}=S; 
       currentNode(I)=S;
    end
    fitnesses=zeros(m,1);
    listAnts=1:m;
    iter=1;
    %----------------------------------------------------------------------
    %create a path for each ant
    while iter<maxIter+1;
        for I=listAnts
            neighbourNodes=getNeighbourNodes(currentNode(I),visitedNodes{I},lx,ly);
            if isempty(neighbourNodes)
                listAnts=listAnts(listAnts~=I);
                if isempty(listAnts)
                    maxIter=0;
                end
            else
                data=[getDistance(currentNode(I),neighbourNodes,Dh,Dv,Ddu,Ddd)', ...
                    pheromonetrail(currentNode(I), neighbourNodes,tauH,tauV,tauDu,tauDd)'];
                Q=data(:,2).^alpha.*data(:,1).^beta;
                p=Q./sum(Q);
                % We select a random Node with probability p
                [p,sortindex]=sort(p,1,'descend');
                neighbourNodes = neighbourNodes(sortindex);
                currentNode(I) = neighbourNodes(min(find(rand()<cumsum(p))));
                %currentNode(I) = neighbourNodes(1);
                visitedNodes{I}=[visitedNodes{I} currentNode(I)];
            end
        end
        iter=iter+1;
    end
    %evaporate
    [tauH,tauV,tauDu,tauDd]=evaporate(tauH,tauV,tauDu,tauDd,rho,p0);
    %----------------------------------------------------------------------
    for I = 1:m
        fitnesses(I)=0;
        nWP=length(visitedNodes{I})-1;
        if nWP==maxIter
            for n=1:nWP
                fitnesses(I) = fitnesses(I)+getDistance(visitedNodes{I}(n),visitedNodes{I}(n+1),Dh,Dv,Ddu,Ddd);
            end
            fitnesses(I) = fitnesses(I)/nWP;
            [tauH,tauV,tauDu,tauDd]=updatePheromone(visitedNodes{I},tauH,tauV,tauDu,tauDd,fitnesses(I),mu);
        end
    %----------------------------------------------------------------------
    end
    %tauH
    
    [currentBestFitness,currentBestK]=max(fitnesses);
    if currentBestFitness>=bestFitness
        bestpath = visitedNodes{currentBestK};
        bestFitness=currentBestFitness;
    end
    [tauH,tauV,tauDu,tauDd]=updatePheromone(bestpath,tauH,tauV,tauDu,tauDd,bestFitness,mu);
    
    %tauH
    %update pheromones=====================================================
    
    
end

% for n =1:length(bestpath)-1
%     getDistance(bestpath(n),bestpath(n+1),Dh,Dv);
% end

%display(visitedNodes{bestK})
indY = ceil(bestpath/lx);
indX = bestpath -(indY-1)*lx;
bestPath(:,1)=x(indX);
bestPath(:,2)=y(indY);


%[Xq,Yq] = meshgrid(x,y);
% figure
% plot(Xq,Yq,'+k');hold on;
% plot(x(indX),y(indY))
% v=[S,E];
% indY = ceil(v/lx);
% indX = v -(indY-1)*lx;
% plot(x(indX),y(indY),'ro')

end

function [tauH,tauV,tauDu,tauDd]=updatePheromone(bestpath,tauH,tauV,tauDu,tauDd,fitness,mu)
[~,lx]=size(tauV);
tauMax=1.5;

for i=1:length(bestpath)-1
    k=min([bestpath(i),bestpath(i+1)]);
    indY = ceil(k/lx);
    indX = k -(indY-1)*lx;
    if abs(bestpath(i)-bestpath(i+1))==1 
        tauH(indY,indX) = min(tauH(indY,indX) + mu*fitness, tauMax);
    elseif abs(bestpath(i)-bestpath(i+1))==lx
        tauV(indY,indX) = min(tauV(indY,indX) + mu*fitness, tauMax);
    elseif abs(bestpath(i)-bestpath(i+1))==lx+1
        tauDu(indY,indX) = min(tauDu(indY,indX) + mu*fitness, tauMax);
    elseif abs(bestpath(i)-bestpath(i+1))==lx-1
        tauDd(indY,indX-1) = min(tauDd(indY,indX-1) + mu*fitness, tauMax);
    end
end

end


function val=pheromonetrail(current,neighbours,tauH,tauV,tauDu,tauDd)
[~,lx]=size(tauV);
val=zeros(1,length(neighbours));

for i=1:length(neighbours)
    k=min([current,neighbours(i)]);
    indY = ceil(k/lx);
    indX = k -(indY-1)*lx;
    
    if abs(current-neighbours(i))==1 
        val(i)=tauH(indY,indX);
    elseif abs(current-neighbours(i))==lx
        val(i)=tauV(indY,indX);
    elseif abs(current-neighbours(i))==lx+1
        val(i)=tauDu(indY,indX);
    elseif abs(current-neighbours(i))==lx-1
        val(i)=tauDd(indY,indX-1);
    end
end

end



function [tauH,tauV,tauDu,tauDd]=evaporate(tauH,tauV,tauDu,tauDd,rho,minTau)

minM=ones(size(tauH))*minTau;
minMd=ones(size(tauDu))*minTau;

tauH=tauH*(1-rho);
tauV=tauV*(1-rho);
tauDu=tauDu*(1-rho);
tauDd=tauDd*(1-rho);

tauH=max(tauH,minM);
tauV=max(tauV,minM');
tauDu=max(tauDu,minMd);
tauDd=max(tauDd,minMd);
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

function neighbourNodes=getNeighbourNodes(k,visitedNodes,lx,ly)
indY = ceil(k/lx);
indX = k -(indY-1)*lx;
if(indX>1&&indX<lx&&indY>1&&indY<ly)
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
neighbourNodes(deleteList)=[];

end


