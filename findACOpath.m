function bestPath= findACOpath(distanceTree, pathLength)

m= 10;               %number of ants.
minPh= 0.01;         %minimum pheromone concentration

mu= 0.0085;
rho= 0.065;
maxPh= 1;

%tree= [(struct.currentNode struct.error struct.pheromone struct.nextNode),...........   ]

bestFitness=0;

%% loop
%Iterate until convergence. Considered convergence when 80% of the total
%ants follow the same path
visitedNodeIndexes= cell(m,1);
mostCommonPath= [];
maxIterations= 250;
iter= 1;
tic
while sum(ismember(cell2mat(visitedNodeIndexes), mostCommonPath, 'rows'))/m < .8 && iter < maxIterations
    %initialization
    currentNodeIndex= ones(m,1);
    [visitedNodeIndexes{1:m}]= deal(1);
    fitnessUpdatePath= cell(m,1);  
    fitnesses= zeros(m,1);
    listAnts=1:m;
    
    %Create a path for each ant
    for I= listAnts
        currentLength=1;
        while currentLength <= pathLength
            nextNodes= distanceTree(currentNodeIndex(I)).nextNode(3,:);
            localIndexes= find(~ismember(nextNodes,visitedNodeIndexes{I})==1);
            neighbourNodeIndexes= nextNodes(~ismember(nextNodes,visitedNodeIndexes{I}));
            
            %[neighbourNodeIndexes, localIndxes]= setdiff(distanceTree(currentNodeIndex(I)).nextNode(3,:), visitedNodeIndexes{I})
            %Handle the case where the ant gets to a dead end path.
            if isempty(neighbourNodeIndexes)
                listAnts= listAnts(listAnts~=I);
                if isempty(listAnts)
                    pathLength=0;
                end
            else
                Q= distanceTree(currentNodeIndex(I)).pheromone(localIndexes);%.^alpha.* distanceTree(currentNodeIndex(I)).error(localIndxes).^beta;
                p= Q./sum(Q);
%                 if any(distanceTree(currentNodeIndex(I)).pheromone(:)== 1)
%                     disp('edge saturated')
%                 end
                
                %We select a random Node with probability p
                [p,sortindex]= sort(p, 2, 'descend');
                %Shuffle together because they need to be in correspondence
                packedIndexes= [neighbourNodeIndexes; localIndexes];
                OrderedNeighbourNodesIndexes = packedIndexes(:,sortindex);
                choosenIdx= OrderedNeighbourNodesIndexes(:, find( rand()< cumsum(p),1));
                fitnesses(I)= fitnesses(I) + distanceTree(currentNodeIndex(I)).error(choosenIdx(2));
                
                %Save outerIdx + innerNextNodeIdx for updating fitness
                fitnessUpdatePath{I}= [fitnessUpdatePath{I} [currentNodeIndex(I); choosenIdx(2)]];
                
                currentNodeIndex(I) = choosenIdx(1);
                visitedNodeIndexes{I}(end+1)= currentNodeIndex(I);
                
            end
            currentLength=currentLength+1;
        end
        fitnesses(I)= fitnesses(I)/pathLength;
        %disp(['Ant ' num2str(I) ' found path with following indexes ' num2str(visitedNodeIndexes{I}) ' and fitness ' num2str(fitnesses(I))])
    end
    %Find best path found so far
    [currentBestFitness,currentBestAnt]= max(fitnesses);
    if currentBestFitness> bestFitness
        %disp(['Best path found so far from ant ' num2str(currentBestAnt) ' with fitness ' num2str(currentBestFitness)])
        bestpathIdxes = visitedNodeIndexes{currentBestAnt};
        bestFitness= currentBestFitness;
    end
    %Evaporate pheromone
    for j= 1: length(distanceTree)
        distanceTree(j).pheromone= max(minPh, (1-rho).* distanceTree(j).pheromone);
    end
    %Update pheromone
    for I = 1:m
        for j=1:length(fitnessUpdatePath{I})
            distanceTree(fitnessUpdatePath{I}(1,j)).pheromone(fitnessUpdatePath{I}(2,j))= ...
                min(maxPh, distanceTree(fitnessUpdatePath{I}(1,j)).pheromone(fitnessUpdatePath{I}(2,j))+ (mu* fitnesses(I)));
            %Extra reinforcement for best tour found so far (elitism)
            if isequal(I, currentBestAnt)
                distanceTree(fitnessUpdatePath{currentBestAnt}(1,j)).pheromone(fitnessUpdatePath{currentBestAnt}(2,j))= ...
                    min(maxPh, distanceTree(fitnessUpdatePath{currentBestAnt}(1,j)).pheromone(fitnessUpdatePath{currentBestAnt}(2,j))+ (mu* fitnesses(currentBestAnt)));
            end
        end
    end
    
    %Compute percentage of converged ants
    [uA,~,uIdx] = unique(cell2mat(visitedNodeIndexes),'rows');
    modeIdx = mode(uIdx);
    mostCommonPath = uA(modeIdx,:);
    iter= iter +1;
    %disp(['-------Percentage of converged ants is  ' num2str((sum(ismember(cell2mat(visitedNodeIndexes), mostCommonPath, 'rows'))/m)*100) ' %'])
end

if iter < maxIterations
    disp(['ACO converged in ' num2str(toc) ' seconds and ' num2str(iter) ' iterations!'])
end
disp([num2str(toc) ' seconds '])
bestPath= cat(1, distanceTree(bestpathIdxes).currentNode);

end
