function test_pathACO(jobID, algorithm, strategy)
%%algorithm: 
%%          -'kriging': uses the kriging error map to move around . 
%%          -'mutualInfo': uses mutual information map to move around

%%strategy: 
%%          -"sampleOnly": samples map in the point corresponding to the maximum error.
%%          -"ACO": uses ACO to find the path that maximizes the mean error within the path
%%          -"greedy": similar to ACO but always chooses the path with the highest error
%%          -"random": samples randomly in the map without considering the error
%%          -"spiral": moves on a spiral path.


%%oss: 
%%      both algorithms use kriging as interpolation strategy.
%%      when using strategy "random" or "spiral" the kind of algorithm used doesn't matter.  
%%      for every run the position of the robot is random and so is the position of the unique static sensor.

close all;
plotOn=0;


% a path of nWPpath waypoints is recomputed each time the robot reaches
% nWayPoints waypoints
nWayPoints=40;% number of waypoints to reach before recomputing the path
nWPpath=20;% number of waypoints in the path
alreadySampled=[];
errorMap=[];

%---------------Load a random field---------------
if isdir('./RandomFields')
    RandStream.setDefaultStream(RandStream('mt19937ar','seed',sum(100*clock)));
    fieldNum= randi([1 100]);
    if mod(jobID, 3)== 1
        field=load(['./RandomFields/RandField_LR_No' num2str(200+fieldNum) '.csv']);
        fieldRange= 40;
    elseif mod(jobID,3)== 2
        field=load(['./RandomFields/RandField_IR_No' num2str(100+fieldNum) '.csv']);
        fieldRange= 20;
    else
        field=load(['./RandomFields/RandField_SR_No' num2str(fieldNum) '.csv']);
        fieldRange= 10;
    end
else
    error('Directory does not exist!!!')
end

%we want a field of 200x200 m. (not 199x199)
field(:,end+1)=field(:,end);
field(end+1,:)=field(end,:);
[Ly,Lx]=size(field);

%position of the stations (static sensors)
stations=[];


if plotOn==1
    subplot(1,3,1);
    plotmap(0:2:Lx-1,0:2:Ly-1,field(1:2:end,1:2:end));
    hold on
    if ~isempty(stations)
        plot(stations(:,1),stations(:,2),'ok','linewidth',2,'MarkerFaceColor','k')
    end
    drawnow
end

%grid used to save the measures
h = 1;
x = 0:h:Lx-1;
y = 0:h:Ly-1;
lx=length(x);
ly=length(y);

%interpolation points (points where the kriging error is computed)
delta=5;
x_=0:delta:Lx-1;
y_=0:delta:Ly-1;

%allowable waypoints position
ph = 5;
px = 0:ph:Lx-1;
py = 0:ph:Ly-1;
lpx=length(px);
lpy=length(py);


%vector that contains the sampling points (nan at position non-sampled)
sVec=ones(lx*ly,1)*nan;
kVec=1:lx*ly;   %indices



%parameters
speedHeli=3.7; %volocity of the quadrotor
measPeriod=3; %sampling period
trendOrder=0; %order of the trend function 0,1 or 2
if strcmp('spiral',strategy)
    pos= 1; 
else
    RandStream.setDefaultStream(RandStream('mt19937ar','seed',sum(100*clock)));
    pos= randi([1 121]);    %initial position
end

Range= 130; % range considered to compute the variogram
posX= mod(pos-1, lpx)* ph;
posY= floor((pos-1)/lpx)* ph;
aX= [max(0, floor(posX-(Range/sqrt(2)))) min(length(x)-1, floor(posX+(Range/sqrt(2))))];
aY= [max(0, floor(posY-(Range/sqrt(2)))) min(length(y)-1, floor(posY+(Range/sqrt(2))))];

RandStream.setDefaultStream(RandStream('mt19937ar','seed',sum(100*clock)));
station= [randi(aX) randi(aY)];
sVec=addSamplingPoints(sVec,[station; posX posY],field,x,y,lx);

fid = fopen('./test.txt','w');%open output file

iter=1;
dist=[];%traveled distance
RMSE=[];%roots mean square error
dist(iter)=0; %traveled distance
h0=0; % distance traveld by the quadrotor since the last waypoint

if strcmp(algorithm, 'mutualInfo')
    [prior,posterior,mutualInfo, temperatureVector]= initializeProbabilities(x_,y_);
end
if strcmp(strategy, 'spiral')
    len= length(px);
    start= 1;
    spiralPath=[];
    while len> start
        spiralPath= [spiralPath; px(start) py(start)];
        spiralPath=[spiralPath; px(start+1:len)' [ones(len-start,1).* py(start)]];
        spiralPath=[spiralPath; [ones(len-start,1).*px(len)] py(start+1:len)'];
        spiralPath=[spiralPath; px(len-1:-1:start)' [ones(len-start,1).*py(len)]];
        spiralPath=[spiralPath; [ones(len-start-1,1).*px(start)] py(len-1:-1:start+1)'];
        start= start+1;
        len= len-1;
    end
    spiralPath= [spiralPath; px(start) py(start)];
end

%% loop
while ((strcmp('ACO', strategy)|| strcmp('greedy',strategy)) && dist(iter)<6040) || ((strcmp('sampleOnly', strategy)...
        || strcmp('random', strategy)) && iter <=150)...
        ||  (strcmp('spiral',strategy) && size(spiralPath,1) > 1)

    %----------------------------------------------------------------------
    %get sampling points
    X=[];
    indK=kVec(~isnan(sVec)); % get the indices of the sampling points
    indY = ceil(indK/lx);
    indX = indK -(indY-1)*lx;
    Y=sVec(indK); % sampled values
    X(:,1)=x(indX); % sampling points position
    X(:,2)=y(indY); % sampling points position
    
    switch algorithm
        case 'kriging'
            %standardisation of the data
            meanV=mean(Y);
            stdV=std(Y);
            Y_=(Y-meanV)/stdV;
            %----------------------------------------------------------------------
            %compute variogram
            % the variogam is computed at discrete positions separated by a
            % distance called lag
            lag=10;
            [fittedModel,fittedParam]=variogram(X,Y_,lag,Range,1);
            
            %----------------------------------------------------------------------
            %kriging interpolation
            [val,krigE_]=kriging(X,Y_,stdV,meanV,fittedModel,fittedParam,trendOrder,x_,y_);
            errorMap= krigE_;
        case 'mutualInfo'
            if ~isempty(alreadySampled)
                [XToBeSampled, indexUniqe]= setdiff(X,alreadySampled,'rows');
                YToBeSampled= Y(indexUniqe);
            else
                XToBeSampled= X;
                YToBeSampled= Y;
            end
            [prior, posterior, mutualInfo]= computePosteriorAndMutualInfo(prior, posterior, mutualInfo, temperatureVector, YToBeSampled, XToBeSampled(:,2), XToBeSampled(:,1), y_, x_, fieldRange);
            %val= sampleTemperatureProbability( posterior, temperatureVector, X(:,2), X(:,1), Y, delta);
            meanV=mean(Y);
            stdV=std(Y);
            Y_=(Y-meanV)/stdV;
            lag=10;
            [fittedModel,fittedParam]=variogram(X,Y_,lag,Range,1);
            [val,~]=kriging(X,Y_,stdV,meanV,fittedModel,fittedParam,trendOrder,x_,y_);
            alreadySampled= [alreadySampled; XToBeSampled];
            errorMap= mutualInfo;
            
    end


    RMSE(iter) = sqrt(mean(mean((val-field(1:delta:lx,1:delta:ly)).^2)));    
    switch strategy
        case 'ACO'
            %compute path normalize between [0,1.5]
            range = max(errorMap(:)) - min(errorMap(:));
            errorMap = (errorMap - min(errorMap(:))) ./ range;
            range2 = 1.5;
            errorMap = (errorMap.* range2);
            [Dh,Dv,Ddu,Ddd]=distanceMatrix(x_,y_,errorMap,px,py);
            [path,~]=findBestPath(px,py,pos,Dh,Dv,Ddu,Ddd,nWPpath,0.6,4.4);
            
            nP=min(nWayPoints+1,length(path));
            path=path(1:nP,:);
            
            indX = floor(path(end,1)/ph)+1;
            indY = floor(path(end,2)/ph)+1;
            pos= indX + (indY-1)*lpx;
            
            [Pts2visit,dist(iter+1),h0] = findPtsAlongPath(path, speedHeli, measPeriod,dist(iter),h0);
        case 'greedy'
            %compute path
            [Dh,Dv,Ddu,Ddd]=distanceMatrix(x_,y_,errorMap,px,py);
            %[path, pos]=findShortestPath(px,py,pos,kMax,Dh,Dv);
            path=greedy(px,py,pos,Dh,Dv,Ddu,Ddd,nWPpath);
            
            nP=min(nWayPoints+1,length(path));
            path=path(1:nP,:);
            
            indX = floor(path(end,1)/ph)+1;
            indY = floor(path(end,2)/ph)+1;
            pos= indX + (indY-1)*lpx;
            
            [Pts2visit,dist(iter+1),h0] = findPtsAlongPath(path, speedHeli, measPeriod,dist(iter),h0);
        case 'sampleOnly'
            [~, idx]= max(errorMap(:));
            [row,col]= ind2sub(size(errorMap), idx);
            Pts2visit= [(col-1)*delta (row-1)*delta];
        case 'random'
            availablePositionMatrix= ones(lx-1,ly-1);
            occupiedPositions= [];
            if ~isempty(stations)
                indX = round(stations(:,1))+1;
                indY = round(stations(:,2))+1;
                stationsIdxX= x(indX)+1;
                stationsIdxY= y(indY)+1;
                occupiedPositions = sub2ind(size(availablePositionMatrix), stationsIdxY', stationsIdxX');
            end
            availablePositionMatrix(occupiedPositions)= 0;
            availablePositionIndexes= find(availablePositionMatrix== 1);
            randIdx= randi([1,size(availablePositionIndexes,1)]);
            availablePositionIndexes(randIdx,1);
            [row, col]= ind2sub(size(availablePositionMatrix), availablePositionIndexes(randIdx,1));
            Pts2visit= [(col-1) (row-1)];
        case 'spiral'
            if size(spiralPath,1)>= nWayPoints
                path= spiralPath(1:nWayPoints,:);
            else
                path= spiralPath(1:end,:);
            end
            [Pts2visit,dist(iter+1),h0] = findPtsAlongPath(path, speedHeli, measPeriod,dist(iter),h0);
            spiralPath= spiralPath(size(path,1)+1:end,:);

    end
    %----------------------------------------------------------------------
    %add new sampling points
    %         if iter==1
    %             sVec=addSamplingPoints(sVec,[0,0],field,x,y,lx);
    %         end
    sVec=addSamplingPoints(sVec,Pts2visit,field,x,y,lx);
    %----------------------------------------------------------------------
    if plotOn==1
        subplot(1,3,2)
        plotmap(x_,y_,val)
        hold on
        plot(X(:,1),X(:,2),'w+','linewidth',2)
        if strcmp('ACO', strategy)
            %plot(path(:,1),path(:,2),'k-o','linewidth',2,'MarkerFaceColor','k')
        end
        drawnow
        hold off
    end
    if plotOn==1
        subplot(1,3,3)
        plotmap(x_,y_,errorMap)
        hold on
        plot(X(:,1),X(:,2),'w+','linewidth',2)
        if strcmp('ACO', strategy)
            plot(path(:,1),path(:,2),'k-o','linewidth',2,'MarkerFaceColor','k')
        end
        drawnow
        hold off
    end
    iter=iter+1;
end
if strcmp(strategy, 'ACO') || strcmp(strategy, 'greedy')
    dist=dist(1:end-1);
    RMSE_=interp1(dist,RMSE,0:50:6000,'linear','extrap');
elseif strcmp(strategy,'spiral')
    dist= dist(1:end-1)
    RMSE_=interp1(dist,RMSE,0:50:dist(end),'linear','extrap');
else
    RMSE_= RMSE;
end

%---------------saves results on file-------------------------
if ~exist('./results', 'dir')
    mkdir('./results');
end

FileName= strcat('./results/SimulationResultJob_', num2str(jobID), '.mat');
save( FileName, 'RMSE_','algorithm','strategy','jobID','fieldNum');

end


%==========================================================================
function [Dh,Dv,Ddu,Ddd]=distanceMatrix(x_,y_,error,x,y)
% create the matrices that contains the distance for the ACO algorithm
% input: x_,y_          grid positions
%        error          Value of the error at the grid position
%        x,y            allowable waypoints positions

Dh=zeros(length(y),length(x)-1);
Dv=zeros(length(y)-1,length(x));
Ddu=zeros(length(y)-1,length(x)-1);
Ddd=zeros(length(y)-1,length(x)-1);

n=10;
dh=(x(2)-x(1))/n;
%val=bilinInterp(f,x,y,x0,y0,lx_1,ly_1);
lx_1=length(x_)-1;
ly_1=length(y_)-1;
for i=1:length(y)
    for j=1:length(x)-1
        %Dh(i,j)=1.5-(krigE(i,j)+krigE(i,j+1));
        for k=0:n
            Dh(i,j) = Dh(i,j) + bilinInterp(x_,y_,error,x(j)+k*dh,y(i),lx_1,ly_1);
        end
        %Dh(i,j) = 1.5 - (Dh(i,j)/5);
        Dh(i,j) = Dh(i,j)/(n+1);
    end
end


for i=1:length(y)-1
    for j=1:length(x)
        %Dv(i,j)=1.5-(krigE(i,j)+krigE(i+1));
        for k=0:n
            Dv(i,j) = Dv(i,j) + bilinInterp(x_,y_,error,x(j),y(i)+k*dh,lx_1,ly_1);
        end
        %Dv(i,j) = 1.5 - (Dv(i,j)/5);
        Dv(i,j) = Dv(i,j)/(n+1);
    end
end

for i=1:length(y)-1
    for j=1:length(x)-1
        %Dv(i,j)=1.5-(krigE(i,j)+krigE(i+1));
        for k=0:n
            Ddu(i,j) = Ddu(i,j) + bilinInterp(x_,y_,error,x(j)+k*dh,y(i)+k*dh,lx_1,ly_1);
            Ddd(i,j) = Ddd(i,j) + bilinInterp(x_,y_,error,x(j)+k*dh,y(i+1)-k*dh,lx_1,ly_1);
        end
        %Dv(i,j) = 1.5 - (Dv(i,j)/5);
        Ddu(i,j) = Ddu(i,j)/(n+1);
        Ddd(i,j) = Ddd(i,j)/(n+1);
    end
end

%norm(p(I,:)-p(J,:))*(0.2+E/30);
end

function plotmap(x,y,map)
% 2d plot
[~, ch]=contourf(x,y,map,30);
set(ch,'edgecolor','none');
set(gca,'FontSize',16)
%xlabel('X [m]','FontSize',14)
%ylabel('Y [m]','FontSize', 14)
%colorbar
axis('equal')
axis([-3 303 -3 303])
end

function sVec=addSamplingPoints(sVec,X,field,x,y,lx)
% addSamplingPoints
% input: sVec           vector that contains the sampling values
%        X              position to  be sampled
%        field          field value
%        x,y            field positions
%        lx             length of the field

for i=1:length(X(:,1))
    indX = round(X(i,1))+1;
    indY = round(X(i,2))+1;
    indK = indX+(indY-1)*lx;
    sVec(indK) = field(y(indY)+1,x(indX)+1);
end
end

function val=bilinInterp(x,y,f,x0,y0,lx_1,ly_1)
% perform bilinear interpolation
% input: x,y            grid positions
%        f              values at grid points
%        x0,y0          position of interest
%        lx_1,ly-1      length of the grid - 1
% ouput: val            interpolation value at the position of interest

h=x(2)-x(1);
Ix1 = round(x0/h);
Iy1 = round(y0/h);
if (Ix1 < 1)
    Ix1 = 1;
elseif (Ix1 > lx_1)
    Ix1 = lx_1;
end

if (Iy1 < 1)
    Iy1 = 1;
elseif (Iy1 > ly_1)
    Iy1 = ly_1;
end

Ix2=Ix1+1;
Iy2=Iy1+1;
DeltaX1 = x0 - x(Ix1);
DeltaX2 = x0 - x(Ix2);
DeltaY1 = y0 - y(Iy1);
DeltaY2 = y0 - y(Iy2);
val=1/(h*h)*(f(Iy1,Ix1)*DeltaX2*DeltaY2 - f(Iy1,Ix2)*DeltaX1*DeltaY2 - f(Iy2,Ix1)*DeltaX2*DeltaY1 + f(Iy2,Ix2)*DeltaX1*DeltaY1);

end