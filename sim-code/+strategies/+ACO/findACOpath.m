function bestPath= findACOpath(nodes,nextNodes,errors,pheromones, pathLength)

m= 20;               %number of ants.
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
listAnts=1:m;

while ((~isempty(listAnts) && sum(ismember(cell2mat(visitedNodeIndexes), mostCommonPath, 'rows'))/length(listAnts) < .8) && iter < maxIterations) || isempty(listAnts)
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

            if isempty(nextNodeIdxs)
                listAnts= listAnts(listAnts~=I);
                if isempty(listAnts)
                    disp('all ants got stuck in a dead end path')
                end
            else
                Q= pheromones(nextNodeIdxs);
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
    for I= listAnts
        pheromones(fitnessUpdatePath{I})= min(maxPh, pheromones(fitnessUpdatePath{I})+ (mu* fitnesses(I)));
        if isequal(I, currentBestAnt)
            pheromones(fitnessUpdatePath{I})= min(maxPh, pheromones(fitnessUpdatePath{I})+ (mu* fitnesses(I)));
        end
    end
    
    %Compute percentage of converged ants
    if ~isempty(listAnts)
        visitedNodeIndexes(cellfun('length',visitedNodeIndexes)~=pathLength+1)=[];
        [uA,~,uIdx] = unique(cell2mat(visitedNodeIndexes),'rows');
        modeIdx = mode(uIdx);
        mostCommonPath = uA(modeIdx,:);
    end
    iter= iter +1;
end
% 
% if iter < maxIterations
%     disp(['ACO converged in ' num2str(toc) ' seconds and ' num2str(iter) ' iterations!'])
% end
% disp([num2str(toc) ' seconds '])
bestPath= nodes(1:2,bestpathIdxes)';

end
