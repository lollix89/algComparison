
function plotSimulation(nRobots, folder)
close all
fileList=dir(strcat('./',folder,'/SimulationResultJob_*.mat'));
shortRange=cell(nRobots);
mediumRange=cell(nRobots);
longRange=cell(nRobots);

%assume L.currentRMSE is a column vector

for i=1:length(fileList)
    L=load(strcat('./',folder,'/', fileList(i).name)); 
    if any(isnan(L.RMSE_))
        disp('there is a nan')
    end
    
    if mod(L.jobID, 3)== 1
        longRange{nRobots}(:, size(longRange{nRobots}, 2)+1)= L.RMSE_;
    elseif mod(L.jobID, 3)== 2
        mediumRange{nRobots}(:, size(mediumRange{nRobots}, 2)+1)= L.RMSE_;
    else
        shortRange{nRobots}(:, size(shortRange{nRobots}, 2)+1)= L.RMSE_;
    end
end

shortRangePlotY=[];
shortRangeSTD= [];
mediumRangePLotY=[];
mediumRangeSTD=[];
longRangePlotY=[];
longRangeSTD=[];
legendNames=[];


for i=1:nRobots
    shortRangePlotY= [shortRangePlotY mean(shortRange{i},2)];
    shortRangeSTD= [shortRangeSTD std(shortRange{i},1,2)];
    
    mediumRangePLotY= [mediumRangePLotY mean(mediumRange{i},2)];
    mediumRangeSTD= [mediumRangeSTD std(mediumRange{i},1,2)];
    
    longRangePlotY= [longRangePlotY mean(longRange{i},2)];
    longRangeSTD=[longRangeSTD std(longRange{i},1,2)];
        
    legendNames=[legendNames; strcat(num2str(i),' robots')];
end

plotRangeX = (0:50:3000)';
if ~exist(['./' folder '/plot'], 'dir')
    mkdir(['./' folder '/plot']);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
shortFigure= figure();
plot(plotRangeX,shortRangePlotY)
title('shortRange RandomField comparison')
ylabel('RMSE')
xlabel('Distance (m)')
grid on
legend(legendNames)
hold on
%adding error bars for STD
for i=1:nRobots
    tmpY= shortRangePlotY(1:5:end,i);
    tmpSTD= shortRangeSTD(1:5:end,i);
    errorbar(plotRangeX(1:5:end), tmpY, tmpSTD, '.k')
end
saveas(shortFigure,['./' folder '/plot/shortRange'],'pdf')
hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mediumFigure=figure();
plot(plotRangeX,mediumRangePLotY)
title('mediumRange RandomField comparison')
ylabel('RMSE')
xlabel('Distance (m)')
grid on
legend(legendNames)
hold on
%adding error bars for STD
for i=1:nRobots
    tmpY= mediumRangePLotY(1:5:end,i);
    tmpSTD= mediumRangeSTD(1:5:end,i);
    errorbar(plotRangeX(1:5:end,i), tmpY, tmpSTD, '.k')
end
saveas(mediumFigure,['./' folder '/plot/mediumRange'],'pdf')
hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
longFigure=figure();
plot(plotRangeX,longRangePlotY)
title('longRange RandomField comparison')
ylabel('RMSE')
xlabel('Distance (m)')
grid on
legend(legendNames)
hold on
%adding error bars for STD
for i=1:nRobots
    tmpY= longRangePlotY(1:5:end,i);
    tmpSTD= longRangeSTD(1:5:end,i);
    errorbar(plotRangeX(1:5:end,i), tmpY, tmpSTD, '.k')
end
saveas(longFigure,['./' folder '/plot/longRange'],'pdf')
hold off


end
