function test_pathACO(algorithm, strategy)
%%algorithm: 
%%          -'kriging': uses the kriging error map to move around . 
%%          -'mutualInfo': uses mutual information map to move around

%%strategy: 
%%          -"sampleOnly": samples map in the point corresponding to the maximum error.
%%          -"ACO": uses ACO to find the path that maximizes the mean error within the path
%%          -"greedy": similar to ACO but always chooses the path with the highest error
%%          -"random": samples randomly in the map without considering the error
%%          -"spiral": moves on a spiral path. (commented)


%%oss: 
%%      both algorithms use kriging as interpolation strategy.
%%      when using strategy "random" or "spiral" the algorithm used doesn't matter.  
%%      for every run the position of the robot is random and so is the position of the unique static sensor.

close all;
plotOn=0;

%---------------Generate a random field---------------
field=fields.gaussian.generate('spherical',300,1,[25 25 0 qrs.config('FieldRange')]);
fieldRange= qrs.config('FieldRange');
[Ly,Lx]=size(field);

%points where the error is computed
delta= 5;
x_= ceil(delta/2)+(0:delta:Lx-1);
y_= ceil(delta/2)+(0:delta:Ly-1);
lx_= size(x_,2);
ly_= size(y_,2);

%Grid containing the sampling points
grid= nan(Lx,Ly);

%Robot parameters:
%   -initial position of the robot
%   -velocity of the quadcopter in m/s
%   -number of allowable directions from current position
%   -horizon distance
%   -sampling period in seconds
%   -traveled distance in meters
%   -sampling offset

posX= randi([1 300]);
posY= randi([1 300]);
speedHeli= 3.7;
allowableDirections= 9;
horizon= 20;
measPeriod= 3;
distance=0;
h0=0;

%simulation parameters:
%   -root mean square error
%   -number of iteration
%   -Range considered to compute the variogram
%   -Static station position which must be in Range within the robot position or it s not
%       possible to compute the variogram.
%   -Number of waypoints to reach before recomputing the path
%   -Number of waypoints in the path

RMSE=[];
iter=1;
Range= 130;
aX= [max(1, floor(posX-(Range/sqrt(2)))) min(Lx, floor(posX+(Range/sqrt(2))))];
aY= [max(1, floor(posY-(Range/sqrt(2)))) min(Ly, floor(posY+(Range/sqrt(2))))];
station= [randi(aX) randi(aY)];
nWayPoints=5;

alreadySampled=[];
errorMap=[];

if plotOn==1
    subplot(1,3,1);
    plotmap(0:2:Lx-1,0:2:Ly-1,field(1:2:end,1:2:end));
    hold on
    if ~isempty(station)
        plot(station(1),station(2),'ok','linewidth',2,'MarkerFaceColor','k')
    end
    drawnow
end

samplingP= [station; round([posX posY])];
grid(sub2ind(size(grid), samplingP(:,2), samplingP(:,1))) = field(sub2ind(size(field), samplingP(:,2), samplingP(:,1)));

if strcmp(algorithm, 'mutualInfo')
    [prior,posterior,mutualInfo, temperatureVector]= initializeProbabilities(lx_,ly_); 
end
% if strcmp(strategy, 'spiral')
%     len= length(px);
%     start= 1;
%     spiralPath=[];
%     while len> start
%         spiralPath= [spiralPath; px(start) py(start)];
%         spiralPath=[spiralPath; px(start+1:len)' [ones(len-start,1).* py(start)]];
%         spiralPath=[spiralPath; [ones(len-start,1).*px(len)] py(start+1:len)'];
%         spiralPath=[spiralPath; px(len-1:-1:start)' [ones(len-start,1).*py(len)]];
%         spiralPath=[spiralPath; [ones(len-start-1,1).*px(start)] py(len-1:-1:start+1)'];
%         start= start+1;
%         len= len-1;
%     end
%     spiralPath= [spiralPath; px(start) py(start)];
% end

%% loop
while ((strcmp('ACO', strategy)|| strcmp('greedy',strategy)) && distance(iter)<3040) ...
        || ((strcmp('sampleOnly', strategy)|| strcmp('random', strategy)) && iter <=150)...
        ||  (strcmp('spiral',strategy) && size(spiralPath,1) > 1)
    display(iter)
    %----------------------------------------------------------------------
    %get sampling points
    X=[];   
    [X(:,2),X(:,1)]= ind2sub(size(grid), find(~isnan(grid)));   %sampling position as x(cols), y(rows)
    Y= grid(sub2ind(size(grid), X(:,2),X(:,1)));                %sampled values 
    
    switch algorithm
        case 'kriging'
            %%
            %kriging error controller
            
            %data standardization
            meanV=mean(Y);
            stdV=std(Y);
            Y_=(Y-meanV)/stdV;
            
            %variogram fitting and kriging error computation
            [fittedModel,fittedParam]= variogram(X,Y_,Range);
            [interpMap,krigE]= kriging(X,Y_,stdV,meanV,fittedModel,fittedParam,x_,y_);
            errorMap= krigE;
            
        case 'mutualInfo'
            %%
            %mutual information controller
            if ~isempty(alreadySampled)
                [samplePositions, indexUniqe]= setdiff(X,alreadySampled,'rows');
                samples= Y(indexUniqe);
            else
                samplePositions= X;
                samples= Y;
            end
            [prior, posterior, mutualInfo]= computePosteriorAndMutualInfo(prior, posterior, mutualInfo, temperatureVector, samples, samplePositions, fieldRange, delta);
            errorMap= mutualInfo;
            alreadySampled= [alreadySampled; samplePositions];
            %Interpolate values using Kriging interpolation algorithm
            meanV= mean(Y);
            stdV= std(Y);
            Y_= (Y-meanV)/stdV;
            [fittedModel,fittedParam]= variogram(X,Y_,Range);
            [interpMap,~]= kriging(X,Y_,stdV,meanV,fittedModel,fittedParam,x_,y_);
            
    end
    
    RMSE(iter) = sqrt(mean(mean((interpMap - field(x_,y_)).^2)));
    switch strategy 
        case 'ACO'
            %%
            %Compute the best path by using the ACO algorithm
            Map= nan(Lx,Ly);
            Map(x_,y_)= errorMap;
            distanceTree= generateACODistanceMatrix(posX, posY, Lx, Ly, Map, allowableDirections, horizon, nWayPoints);
            path= findACOpath(distanceTree, nWayPoints); 
            posX= path(end,1);
            posY= path(end,2);
            
            [Pts2visit, distance(iter+1), h0] = findPtsAlongPath(path, speedHeli, measPeriod,distance(iter),h0);
        case 'greedy'
            %%
            % Find a path in the field by using a greedy strategy that
            % chooses the edge with the highest mean error.
            
            Map= nan(Lx,Ly);
            Map(x_,y_)= errorMap;
                
            path= greedy(posX, posY, Lx, Ly, Map, allowableDirections, horizon, nWayPoints); 
            posX= path(end,1);
            posY= path(end,2);
            
            [Pts2visit, distance(iter+1), h0] = findPtsAlongPath(path, speedHeli, measPeriod, distance(iter),h0);
        case 'sampleOnly'
            %%
            %Assume the robot moves with infinite velocity to the point
            %where the error is the highest, therefore, this strategy
            %doesn't need a moving plan.
            Map= nan(Lx,Ly);
            Map(x_,y_)= errorMap; 
            [~, idx]= max(Map(:));
            [y,x]= ind2sub(size(Map), idx);
            Pts2visit= [x y];
        case 'random'
            %%
            %All the strategy are disregarded and the robot moves with
            %infinity velocity to a random position, this strategy doesn't
            %need a moving plan.
            availablePositionMatrix= ones(Lx,Ly);
            if ~isempty(station)
                availablePositionMatrix(sub2ind(size(availablePositionMatrix), station(:,2), station(:,1)))= 0;
            end
            availablePositionIndexes= find(availablePositionMatrix== 1);
            randIdx= randi([1,size(availablePositionIndexes,1)]);
            availablePositionIndexes(randIdx,1);
            [y, x]= ind2sub(size(availablePositionMatrix), availablePositionIndexes(randIdx,1));
            Pts2visit= [x y];
%         case 'spiral'
%             if size(spiralPath,1)>= nWayPoints
%                 path= spiralPath(1:nWayPoints,:);
%             else
%                 path= spiralPath(1:end,:);
%             end
%             [Pts2visit,distance(iter+1),h0] = findPtsAlongPath(path, speedHeli, measPeriod,distance(iter),h0);
%             spiralPath= spiralPath(size(path,1)+1:end,:);

    end
    %---------------------------Add sampled points to the grid-------------------------------------------
    if ~isempty(Pts2visit)
        grid(sub2ind(size(grid), Pts2visit(:,2), Pts2visit(:,1))) = field(sub2ind(size(field), Pts2visit(:,2), Pts2visit(:,1)));
    end
    %---------------------------Plots-------------------------------------------
    if plotOn==1
        %Plot the interpolated maps together with the sampling points and
        %the path of the robot.
        subplot(1,3,2)
        plotmap(x_,y_,interpMap)
        hold on
        plot(X(:,1),X(:,2),'w+','linewidth',2)
        if strcmp('ACO', strategy) || strcmp('greedy', strategy)
            plot(path(:,1),path(:,2),'k-o','linewidth',2,'MarkerFaceColor','k')
        end
        hold off
        
        %Plot the error map together with the sampling points and the path
        %of the robot.
        subplot(1,3,3)
        plotmap(x_,y_,errorMap)
        hold on
        plot(X(:,1),X(:,2),'w+','linewidth',2)
        if strcmp('ACO', strategy) || strcmp('greedy', strategy)
            plot(path(:,1),path(:,2),'k-o','linewidth',2,'MarkerFaceColor','k')
        end
        drawnow
        hold off
    end
    iter=iter+1;
end
%%Interpolate the RMSE and save it to a file.
if strcmp(strategy, 'ACO') || strcmp(strategy,'greedy')
    distance=distance(1:end-1);
    RMSE=interp1(distance,RMSE,0:50:3000,'linear','extrap');
% elseif strcmp(strategy,'spiral')
%     distance= distance(1:end-1);
%     RMSE_=interp1(distance,RMSE,0:50:distance(end),'linear','extrap');

end
csvwrite('test.csv', RMSE);
%movie2avi(F, 'movie.avi', 'compression','None', 'fps',0.5);
end


function plotmap(x,y,map)
% 2d plot
[~, ch]=contourf(x,y,map,30);
set(ch,'edgecolor','none');
set(gca,'FontSize',16)
axis('equal')
axis([-3 303 -3 303])
end
