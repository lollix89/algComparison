function bestPath= findACOpath(nodes,nextNodes,errors,pheromones, pathLength)

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
            endChildIdx= sum(nodes(3,1:currentNodeIndex(I)));
            nextNodeIdxs= endChildIdx-nodes(3,currentNodeIndex(I))+1:endChildIdx;
            %nextNodes= distanceTree(currentNodeIndex(I)).nextNode(3,:);
%             neighbourNodeIndexes= nextNodes;
%             localIndexes= 1:length(neighbourNodeIndexes);     
%              localIndexes= find(~ismember(nextNodes,visitedNodeIndexes{I})==1);
%              neighbourNodeIndexes= nextNodes(~ismember(nextNodes,visitedNodeIndexes{I}));

            
            %[neighbourNodeIndexes, localIndxes]= setdiff(distanceTree(currentNodeIndex(I)).nextNode(3,:), visitedNodeIndexes{I})
            %Handle the case where the ant gets to a dead end path.
            if isempty(nextNodeIdxs)
                listAnts= listAnts(listAnts~=I);
                if isempty(listAnts)
                    pathLength=0;
                end
            else
                Q= pheromones(nextNodeIdxs);%.^alpha.* distanceTree(currentNodeIndex(I)).error(localIndxes).^beta;
                p= Q./sum(Q);
%                 if any(distanceTree(currentNodeIndex(I)).pheromone(:)== 1)
%                     disp('edge saturated')
%                 end
                
                %We select a random Node with probability p
                [p,sortindex]= sort(p, 2, 'descend');
                %Shuffle together because they need to be in correspondence
                OrderednextNodeIdxs = nextNodeIdxs(sortindex);
                nextNodeIdxIdx= OrderednextNodeIdxs(:, find( rand()< cumsum(p),1));
                nextNodeIdx= nextNodes(nextNodeIdxIdx);
                fitnesses(I)= fitnesses(I) + errors(nextNodeIdxIdx);
                
                %Save for updating fitness
                fitnessUpdatePath{I}(end+1)= nextNodeIdxIdx;
                
                currentNodeIndex(I) = nextNodeIdx;
                visitedNodeIndexes{I}(end+1)= currentNodeIndex(I);
                
            end
            currentLength=currentLength+1;
        end
        fitnesses(I)= fitnesses(I)/pathLength;
    end
    %Find best path found so far
    [currentBestFitness,currentBestAnt]= max(fitnesses);
    if currentBestFitness> bestFitness
        bestpathIdxes = visitedNodeIndexes{currentBestAnt};
        bestFitness= currentBestFitness;
    end
    %Evaporate pheromone
    pheromones= max(minPh, (1-rho).* pheromones);
    
    %Update pheromone    
    for I= 1:m
        pheromones(fitnessUpdatePath{I})= min(maxPh, pheromones(fitnessUpdatePath{I})+ (mu* fitnesses(I)));
        if isequal(I, currentBestAnt)
            pheromones(fitnessUpdatePath{I})= min(maxPh, pheromones(fitnessUpdatePath{I})+ (mu* fitnesses(I)));
        end
    end
    
    %Compute percentage of converged ants
    [uA,~,uIdx] = unique(cell2mat(visitedNodeIndexes),'rows');
    modeIdx = mode(uIdx);
    mostCommonPath = uA(modeIdx,:);
    iter= iter +1;
end
% 
% if iter < maxIterations
%     disp(['ACO converged in ' num2str(toc) ' seconds and ' num2str(iter) ' iterations!'])
% end
% disp([num2str(toc) ' seconds '])
bestPath= nodes(1:2,bestpathIdxes)';

end
