function [measPts,dist,hend] = findPtsAlongPath(coords,speed,measPeriod,dist,h0)
% This function finds all sampling points on a given path considering
% a given sampling period and a fixed quadrotor speed.
% Every time the quadrotor travels a distance corresponding to its speed
% times sampling period a new point along the path is added to the list. 

% INPUT
% Coords        : coordinates of the calculated path (Nx2)
% speed         : speed of the quadrotor (in m/s)
% measPeriod    : the sampling time
% dist          : the distance travelled so far
% h0            : The left distance for next iteration

% OUTPUTS
% measPts       : Points that will be sampled
% dist          : distance travelled since the beginning of the simulation

h= speed*measPeriod;
    
%part of the measurment distance astride two intervals, initialy = 0.
offset= h0;
m=1;
measPts=[];

for i=1:length(coords)-1 
    v=coords(i+1,:)-coords(i,:);
    l=norm(v);
    dist=dist+l;
    v=v/l;
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