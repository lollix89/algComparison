function [measPts,dist,hend] = findPtsAlongPath(coords,speed,measPeriod,dist,h0)
% findPtsAlongPath Finds all sampling points on a given path considering
% a given sampling period and quadrotor speed.
% Every time the quadrotor travels a distance corresponding to its speed
% times sampling period a new point along the path is added to the list. 

% INPUT
% Coords     : coordinates of the calculated path
% speed      : speed of the quadrotor (in m/s)
% totDist    : the total distance of the calculated path
% maxDist    : the maximal distance that will be travelled along the path

% OUTPUTS
% measPts : Points that will be measured


h=speed*measPeriod;%sampling distance
%nMeasPts=floor(totDist/h)%number of measurment points
%measPts=zeros(nMeasPts,2);

%part of the measurment distance astride two intervals, initialy = h since
%we don't want to take the initial position as a measurment point
offset=h0;
m=1;
for i=1:length(coords)-1 %loop over the path positions
    v=coords(i+1,:)-coords(i,:);
    l=norm(v);
    dist=dist+l;
    v=v/l; %normalize
    k=0;
    while (offset+k*h)<=l;
        measPts(m,:) = coords(i,:)+(offset+k*h)*v;
        m=m+1;
        k=k+1;
    end
    offset=offset+k*h-l;%new offset
end

hend=offset;


% offset=h0;
% m=1;
% for i=1:length(coords)-1 %loop over the path positions
%     v=coords(i+1,:)-coords(i,:);
%     l=norm(v);
%     dist=dist+l;
%     v=v/l; %normalize
%     n=floor((l-offset)/h); %number of sampling distance in the interval
%     for k=0:n;
%         measPts(m,:) = coords(i,:)+(offset+k*h)*v;
%         m=m+1;
%     end
%     offset=h -((l-offset)-n*h);%new offset
% end
% 
% hend=h0;
%display(m-1)

% figure
% plot(coords(:,1), coords(:,2),'-r','LineWidth',2);hold on;
% plot(coords(:,1), coords(:,2), 'or','LineWidth',3)
% plot(measPts(:,1), measPts(:,2), '.k','LineWidth',3)
% axis equal
% xlabel('Coordinate X [m]','FontSize',14)
% set(gca,'FontSize',14)
% ylabel('Coordinate Y [m]','FontSize', 14)


end