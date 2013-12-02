function bestPath= findACOpath(distanceTree, maxIterations, alpha, beta)


m= 20;               %number of ants.
p0= 0.01;            %initial pheromone concentration
minPh= 0.001;

mu= 0.01;
rho= 0.11;
maxPh= 1;

%tree= [(struct.currentNode struct.error struct.pheromone struct.nextNode),...........   ]

bestFitness=0;

%% loop
for K=1:50
    %initialization
    %disp(['Iteration ' num2str(K)])
    visitedNodeIndexes= cell(m,1);
    currentNodeIndex= ones(m,1);
    [visitedNodeIndexes{1:m}]= deal(1);
    fitnessUpdatePath= cell(m,1);
    
    fitnesses= zeros(m,1);
    listAnts=1:m;
    %Create a path for each ant
    for I= listAnts
        iter=1;
        while iter < maxIterations
            [neighbourNodeIndexes, localIndxes]= setdiff(distanceTree(currentNodeIndex(I)).nextNode(3,:), visitedNodeIndexes{I});
            %Handle the case where the ant gets to a dead end path.
            if isempty(neighbourNodeIndexes)
                listAnts= listAnts(listAnts~=I);
                if isempty(listAnts)
                    maxIterations=0;
                end
            else
                %alpha accounts for the pheromone
                %beta accounts for the visibility
%                 %DEBUG
%                 if currentNodeIndex(I)==1 && I==1
%                     distanceTree(currentNodeIndex(I)).pheromone(localIndxes)
%                 end
%                 if 
                %
                Q= distanceTree(currentNodeIndex(I)).pheromone(localIndxes);%.^alpha.* distanceTree(currentNodeIndex(I)).error(localIndxes).^beta;
                p= Q./sum(Q);
                %We select a random Node with probability p
                [p,sortindex]= sort(p, 2, 'descend');
                %shuffle together because they need to be in correspondence
                packedIndexes= [neighbourNodeIndexes; localIndxes];
                
                OrderedNeighbourNodesIndexes = packedIndexes(:,sortindex);
                
                choosenIdx= OrderedNeighbourNodesIndexes(:, find( rand()< cumsum(p),1));
                fitnesses(I)= fitnesses(I) + distanceTree(currentNodeIndex(I)).error(choosenIdx(2));
                
                %save outerIdx + innerNextNodeIdx for updating fitness
                fitnessUpdatePath{I}= [fitnessUpdatePath{I} [currentNodeIndex(I); choosenIdx(2)]];
                
                currentNodeIndex(I) = choosenIdx(1);
                visitedNodeIndexes{I}(end+1)= currentNodeIndex(I);
                
            end
            iter=iter+1;
        end
            %disp(['Ant ' num2str(I) ' found path with following indexes ' num2str(visitedNodeIndexes{I})])
    end
    %evaporate
    for j= 1: length(distanceTree)
        distanceTree(j).pheromone(:)= max(minPh, (1-rho).* distanceTree(j).pheromone(:));
    end
    %update pheromone
    for I = 1:m
        for j=1:length(fitnessUpdatePath{I})
            distanceTree(fitnessUpdatePath{I}(1,j)).pheromone(fitnessUpdatePath{I}(2,j))= ...
                min(maxPh, distanceTree(fitnessUpdatePath{I}(1,j)).pheromone(fitnessUpdatePath{I}(2,j))+ (mu* fitnesses(I)));
        end
    end
    %debug check convergence. Considered converged if 80% of the total ants
    %followed the same path
    [uA,~,uIdx] = unique(cell2mat(visitedNodeIndexes),'rows');
    modeIdx = mode(uIdx);
    mostCommonPath = uA(modeIdx,:);
    
    %mostCommonPath= mode(cell2mat(visitedNodeIndexes));
    %disp(['---->Percentage of converged ants is  ' num2str(sum(ismember(cell2mat(visitedNodeIndexes), mostCommonPath, 'rows'))/m)])
    if sum(ismember(cell2mat(visitedNodeIndexes), mostCommonPath, 'rows'))/m > .8
        disp('converged!!')
    end  
    %
    
    fitnesses= fitnesses./(maxIterations-1);
    [currentBestFitness,currentBestAnt]= max(fitnesses);
    if currentBestFitness> bestFitness
        %disp(['Best path found so far from ant ' num2str(currentBestAnt) ' with fitness ' num2str(currentBestFitness)])
        bestpathIdxes = visitedNodeIndexes{currentBestAnt};
        bestFitness= currentBestFitness;
    end
end

bestPath= cat(1, distanceTree(bestpathIdxes).currentNode);



end
