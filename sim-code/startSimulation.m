function startSimulation(algorithm, strategy)
%%ALGORITHM:
%%          -'kriging': uses the kriging error map to move around .
%%          -'mutualInfo': uses mutual information map to move around

%%STRATEGY:
%%          -"sample": samples map in the point corresponding to the maximum error.
%%          -"ACO": uses ACO to find the path that maximizes the mean error within the path
%%          -"greedy": similar to ACO but always chooses the path with the highest error
%%          -"random": samples randomly in the map without considering the error


%%OSS:
%%      both algorithms use kriging as interpolation strategy.
%%      when using strategy "random" the algorithm used doesn't matter.
%%      for every run the position of the robot is random and so is the position of the unique static sensor.
close all;
plotOn= 0;

%Generate a random field
field= fields.gaussian.generate(qrs.config('FieldModel'),qrs.config('Size'),1,[25 25 0 qrs.config('FieldRange')]);

[Ly,Lx]= size(field);

%Points where the error is computed
delta= 5;
x_= ceil(delta/2)+(0:delta:Lx-1);
y_= ceil(delta/2)+(0:delta:Ly-1);
lx_= size(x_,2);
ly_= size(y_,2);

%Robot parameters:
%   -initial position of the robot
%   -velocity of the quadcopter in m/s
%   -number of allowable directions from current position
%   -horizon distance
%   -sampling period in seconds
%   -traveled distance in meters
%   -sampling offset

posX= randi([1 Lx]);
posY= randi([1 Ly]);
speedHeli= 3.7;
allowableDirections= qrs.config('AllowedDirections');
horizon= qrs.config('Horizon');
measPeriod= 3;
distance= 0;
h0= 0;

%Simulation parameters:
%   -root mean square error
%   -number of iteration
%   -Range considered to compute the variogram
%   -Static station position which must be in Range within the robot position or it s not
%       possible to compute the variogram.
%   -Number of waypoints in the path

RMSE=[];
iter=1;
Range= 130;
aX= [max(1, floor(posX-(Range/sqrt(2)))) min(Lx, floor(posX+(Range/sqrt(2))))];
aY= [max(1, floor(posY-(Range/sqrt(2)))) min(Ly, floor(posY+(Range/sqrt(2))))];
station= [randi(aX) randi(aY)];
nWayPoints= 5;
travellingDistance= 5000;
numberOfSamplings= 300;

if plotOn==1
    subplot(1,3,1);
    plotmap(0:2:Lx-1,0:2:Ly-1,field(1:2:end,1:2:end));
    hold on
    if ~isempty(station)
        plot(station(1),station(2),'ok','linewidth',2,'MarkerFaceColor','k')
    end
    drawnow
end

Pts2Sample= [station; round([posX posY])];      %sampling position as x(cols), y(rows)
valuePts2Sample= field(sub2ind(size(field), Pts2Sample(:,2), Pts2Sample(:,1)));
samplePntHistory= Pts2Sample;
sampleValueHistory= valuePts2Sample;

if strcmp(algorithm, 'mutualInfo')
    [prior,posterior,mutualInfo, tVector]= mutual.initializeProbabilities(lx_, ly_);
end

%% loop
while ((strcmp('ACO', strategy)|| strcmp('greedy',strategy)) && distance(iter)< travellingDistance) ...
        || ((strcmp('sample', strategy)|| strcmp('random', strategy)) && iter <= numberOfSamplings)
    %display(iter)
    
    switch algorithm
        case 'kriging'
            %%
            %Kriging error controller
            %data standardization
            meanV=mean(sampleValueHistory);
            stdV=std(sampleValueHistory);
            Y_=(sampleValueHistory-meanV)/stdV;
            
            %Variogram fitting and kriging error computation. If estimation
            %is true we estimate the parameters, otherwise we just take
            %those provided.
            if isequal(qrs.config('Estimation'),1)
                [fittedModel,Param]= kriging.variogram(samplePntHistory,Y_,Range);
            else
               fittedModel= @(param,h) ((h<param(2))*0.5.*(3*h/(param(2))-(h/param(2)).^3) + (h>=param(2)))*(param(1));
               stdV= qrs.config('Sill');
               Param= [1 qrs.config('Range')];
            end
            [interpMap,krigE]= kriging.computeKriging(samplePntHistory,Y_,stdV,meanV,fittedModel,Param,x_,y_);
            errorMap= krigE;
            
        case 'mutualInfo'
            %%
            %Mutual information controller.  If estimation
            %is true we estimate the parameters, otherwise we just take
            %those provided.
            if isequal(qrs.config('Estimation'),1)
                meanV=mean(sampleValueHistory);
                stdV=std(sampleValueHistory);
                Y_=(sampleValueHistory-meanV)/stdV;
                [~,Param]= kriging.variogram(samplePntHistory,Y_,Range);
                Param= Param.*stdV+meanV;
            else
               Param= [qrs.config('Sill') qrs.config('Range')];
            end
            Param= num2cell(Param);
            Param{end+1}= qrs.config('Function');
            [prior, posterior, mutualInfo]= mutual.computePosteriorAndMutualInfo(prior, posterior, mutualInfo, tVector, valuePts2Sample, Pts2Sample, Param, delta);
            errorMap= mutualInfo;
            %Interpolate values using Kriging interpolation algorithm
            meanV= mean(sampleValueHistory);
            stdV= std(sampleValueHistory);
            Y_= (sampleValueHistory-meanV)/stdV;
            [fittedModel,Param]= kriging.variogram(samplePntHistory,Y_,Range);
            [interpMap,~]= kriging.computeKriging(samplePntHistory,Y_,stdV,meanV,fittedModel,Param,x_,y_);
        otherwise
            error('Wrong algorithm, possible values are [kriging mutualInfo]')
    end
    
    RMSE(iter) = sqrt(mean(mean((interpMap - field(x_,y_)).^2)));
    switch strategy
        case 'ACO'
            %%
            %Compute the best path by using the ACO algorithm
            Map= nan(Lx,Ly);
            Map(x_,y_)= errorMap;
            [nodes,nextNodeIdxs,errors,pheromones]= strategies.ACO.generateACODistanceMatrix(posX, posY, Lx, Ly, Map, allowableDirections, horizon, nWayPoints);
            Path= strategies.ACO.findACOpath(nodes,nextNodeIdxs,errors,pheromones, nWayPoints);
            posX= Path(end,1);
            posY= Path(end,2); 
            [Pts2Sample, distance(iter+1), h0] = strategies.findPtsAlongPath(Path, speedHeli, measPeriod,distance(iter),h0);
        case 'greedy'
            %%
            % Find a path in the field by using a greedy strategy that
            % chooses the edge with the highest mean error.  
            Map= nan(Lx,Ly);
            Map(x_,y_)= errorMap;
            Path= strategies.greedy.greedy(posX, posY, Lx, Ly, Map, allowableDirections, horizon, nWayPoints);
            posX= Path(end,1);
            posY= Path(end,2);          
            [Pts2Sample, distance(iter+1), h0] = strategies.findPtsAlongPath(Path, speedHeli, measPeriod, distance(iter),h0);
        case 'sample'
            %%
            %Assume the robot moves with infinite velocity to the point
            %where the error is the highest, therefore, this strategy
            %doesn't need a moving strategy.
            Map= nan(Lx,Ly);
            Map(x_,y_)= errorMap;
            [~, idx]= max(Map(:));
            [y,x]= ind2sub(size(Map), idx);
            posX= x;
            posY= y;
            Pts2Sample= [x y];
        case 'random'
            %%
            %All the strategy are disregarded and the robot moves with
            %infinity velocity to a random position, this strategy doesn't
            %need a moving strategy.
            availablePositionMatrix= ones(Lx,Ly);
            if ~isempty(station)
                availablePositionMatrix(sub2ind(size(availablePositionMatrix), station(:,2), station(:,1)))= 0;
            end
            availablePositionIndexes= find(availablePositionMatrix== 1);
            randIdx= randi([1,size(availablePositionIndexes,1)]);
            availablePositionIndexes(randIdx,1);
            [y, x]= ind2sub(size(availablePositionMatrix), availablePositionIndexes(randIdx,1));
            posX= x;
            posY= y;
            Pts2Sample= [x y];
        otherwise
            error('Wrong strategy, possible values are [greedy ACO sample random]')    
    end
    %Add sampled points to the grid
    if ~isempty(Pts2Sample)
        valuePts2Sample= field(sub2ind(size(field), Pts2Sample(:,2), Pts2Sample(:,1)));
        samplePntHistory= [samplePntHistory; Pts2Sample];
        [samplePntHistory,idx,~]= unique(samplePntHistory,'stable','rows');
        sampleValueHistory= [sampleValueHistory; valuePts2Sample];
        sampleValueHistory= sampleValueHistory(idx);
    end
    %Plots
    if plotOn==1
        %Plot the interpolated maps together with the sampling points and
        %the path of the robot.
        subplot(1,3,2)
        plotmap(x_,y_,interpMap)
        hold on
        plot(samplePntHistory(:,1),samplePntHistory(:,2),'w+','linewidth',2)
        if strcmp('ACO', strategy) || strcmp('greedy', strategy)
            plot(Path(:,1),Path(:,2),'k-o','linewidth',2,'MarkerFaceColor','k')
        end
        hold off
        
        %Plot the error map together with the sampling points and the path
        %of the robot.
        subplot(1,3,3)
        plotmap(x_,y_,errorMap)
        hold on
        plot(samplePntHistory(:,1),samplePntHistory(:,2),'w+','linewidth',2)
        if strcmp('ACO', strategy) || strcmp('greedy', strategy)
            plot(Path(:,1),Path(:,2),'k-o','linewidth',2,'MarkerFaceColor','k')
        end
        drawnow
        hold off
    end
    iter=iter+1;
end
saveResults(strategy, distance, RMSE, travellingDistance, numberOfSamplings)
%movie2avi(F, 'movie.avi', 'compression','None', 'fps',0.5);
end


%%2d plotting
function plotmap(x,y,map)
[~, ch]=contourf(x,y,map,30);
set(ch,'edgecolor','none');
set(gca,'FontSize',16)
axis('equal')
axis([-3 qrs.config('Size')+3 -3 qrs.config('Size')+3])
end

function saveResults(strategy, distance, RMSE, travellingDistance, numberOfSamplings)
%%Save the simulation results to a file

if strcmp(strategy, 'ACO') || strcmp(strategy,'greedy')
    distance= distance(1:end-1);
    RMSE=interp1(distance,RMSE,0:50:travellingDistance,'linear','extrap');
    dlmwrite([qrs.config('DataDirectory') '_' num2str(randi(1e+10,1))],[0:50:travellingDistance; RMSE]','-append');
else
    dlmwrite([qrs.config('DataDirectory') '_' num2str(randi(1e+10,1))],[1:numberOfSamplings; RMSE]', '-append');
end

end
