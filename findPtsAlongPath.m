function [measPts,dist,hend] = findPtsAlongPath(coords,speed,measPeriod,dist,h0)
% findPtsAlongPath Finds all sampling points on a given path considering
% a given sampling period and quadrotor speed.
% Every time the quadrotor travels a distance corresponding to its speed
% times sampling period a new point along the path is added to the list. 

% INPUT
% Coords     : coordinates of the calculated path (Nx2)
% speed      : speed of the quadrotor (in m/s)
% totDist    : the total distance of the calculated path
% maxDist    : the maximal distance that will be travelled along the path

% OUTPUTS
% measPts : Points that will be measured
% dist : distance since the beginning of the simulation


h=speed*measPeriod;%sampling distance

%part of the measurment distance astride two intervals, initialy = h since
%we don't want to take the initial position as a measurment point
offset= h0;
m=1;
measPts=[];

for i=1:length(coords)-1 
    v=coords(i+1,:)-coords(i,:);
    l=norm(v);
    dist=dist+l;
    v=v/l; %get direction
    k=0;
    while (offset+k*h)<= l;
        measPts(m,:) = coords(i,:)+(offset+k*h)*v;
        m=m+1;
        k=k+1;
    end
    offset=offset+k*h-l;
end

hend=offset;
measPts= round(measPts);

end