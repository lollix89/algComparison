
function plotSimulationComparison(nRobots, folders)
close all

fileList={};
ComparisonNumber= size(folders,2);

for i=1: ComparisonNumber
    fileList{i}= dir(strcat('./', folders{i}, '/SimulationResultJob_*.mat'));
end

shortRange=cell(ComparisonNumber,nRobots);
mediumRange=cell(ComparisonNumber,nRobots);
longRange=cell(ComparisonNumber,nRobots);

for j=1: ComparisonNumber
    for i=1:length(fileList{j})
        L=load(strcat('./', folders{j},'/', fileList{j}(i).name));
        
        if mod(L.jobID, 3)== 1
            longRange{j,nRobots}(:, size(longRange{j,nRobots}, 2)+1)= L.RMSE_;
        elseif mod(L.jobID, 3)== 2
            mediumRange{j,nRobots}(:, size(mediumRange{j,nRobots}, 2)+1)= L.RMSE_;
        else
            shortRange{j,nRobots}(:, size(shortRange{j,nRobots}, 2)+1)= L.RMSE_;
        end
    end
end

shortRangePlotY= cell(1,ComparisonNumber);
shortRangeSTD= cell(1,ComparisonNumber);
mediumRangePLotY= cell(1,ComparisonNumber);
mediumRangeSTD= cell(1,ComparisonNumber);
longRangePlotY= cell(1,ComparisonNumber);
longRangeSTD= cell(1,ComparisonNumber);

for j=1:ComparisonNumber
    for i=1:nRobots
        shortRangePlotY{j}= [shortRangePlotY{j} mean(shortRange{j,i},2)];
        shortRangeSTD{j}= [shortRangeSTD{j} std(shortRange{j,i},1,2)];
        
        mediumRangePLotY{j}= [mediumRangePLotY{j} mean(mediumRange{j,i},2)];
        mediumRangeSTD{j}= [mediumRangeSTD{j} std(mediumRange{j,i},1,2)];
        
        longRangePlotY{j}= [longRangePlotY{j} mean(longRange{j,i},2)];
        longRangeSTD{j}=[longRangeSTD{j} std(longRange{j,i},1,2)];

    end
end

destFolderName= 'comparison-';
for i=1:ComparisonNumber
   destFolderName= strcat(destFolderName, folders{i});
end


plotRangeX = (0:50:3000)';
%plotRangeX= (1:150)';
plotRangeXBar= repmat(plotRangeX(1:10:end), 1, ComparisonNumber);

if ~exist(strcat('./', destFolderName), 'dir')
    mkdir(strcat('./', destFolderName));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isempty(shortRangePlotY{1})~=1 && isempty(shortRangePlotY{2})~=1
    
    shortRangePlotY= cell2mat(shortRangePlotY);
    shortRangeSTD= cell2mat(shortRangeSTD);
    
    shortFigure= figure(1);
    plot(plotRangeX,shortRangePlotY)
    title('ShortRange RandomField comparison')
    ylabel('RMSE')
    xlabel('sample')
    grid on
    legend(folders)
    hold on
%     
    tmpY= shortRangePlotY(1:10:end,:);
    tmpSTD= shortRangeSTD(1:10:end,:);
    errorbar(plotRangeXBar, tmpY, tmpSTD, '.')

    saveas(shortFigure,strcat('./', destFolderName,'/shortRange'),'pdf')
    hold off
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isempty(mediumRangePLotY{1})~=1
    mediumRangePLotY= cell2mat(mediumRangePLotY);
    mediumRangeSTD= cell2mat(mediumRangeSTD);
    mediumFigure=figure(2);
    plot(plotRangeX,mediumRangePLotY)
    title('MediumRange RandomField comparison')
    ylabel('RMSE')
    xlabel('sample')
    grid on
    legend(folders)
    hold on
    
    tmpY= mediumRangePLotY(1:10:end,:);
    tmpSTD= mediumRangeSTD(1:10:end,:);
    %plotRangeX= repmat(plotRangeX(1:5:end), 1, ComparisonNumber);
    errorbar(plotRangeXBar, tmpY, tmpSTD, '.')
    
    saveas(mediumFigure,strcat('./', destFolderName,'/mediumRange'),'pdf')
    hold off
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isempty(longRangePlotY{1})~=1
    longRangePlotY= cell2mat(longRangePlotY);
    longRangeSTD= cell2mat(longRangeSTD);
    longFigure=figure(3);
    plot(plotRangeX,longRangePlotY)
    title('longRange RandomField comparison')
    ylabel('RMSE')
    xlabel('sample')
    grid on
    legend(folders)
    hold on

    tmpY= longRangePlotY(1:10:end,:);
    tmpSTD= longRangeSTD(1:10:end,:);
    %plotRangeX= repmat(plotRangeX(1:5:end), 1, ComparisonNumber);
    errorbar(plotRangeXBar, tmpY, tmpSTD, '.')
    
    saveas(longFigure,strcat('./', destFolderName,'/longRange'),'pdf')
    hold off
end


end
