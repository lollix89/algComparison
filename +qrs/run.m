function run(varargin)
% QRS.RUN execute quadrotor simulation
%   QRS.RUN(C1,V1,...,CN,VN) starts a simulation with a configuration
%   specified by the provided C = V pairs of config keys and values.

clearvars -global QRS_CONFIG
qrs.configuration.set(varargin{:});

% initialize RNGs
if isKey(qrs.config,'RandomSeed')
	rng(qrs.config('RandomSeed'));
	qrs.r.init(qrs.config('RandomSeed'));
else
	qrs.r.init;
end

%**************Get config parameters or set to default
if ~isKey(qrs.config,'Horizon')
    qrs.configuration.set('Horizon',20);
end
if ~isKey(qrs.config,'AllowedDirections')
    qrs.configuration.set('AllowedDirections',8);
end
if ~isKey(qrs.config,'Sill')
    qrs.configuration.set('Sill',25);
end
if ~isKey(qrs.config,'Function')
    qrs.configuration.set('Function','linear');
end
if ~isKey(qrs.config,'Algorithm')
    qrs.configuration.set('Algorithm','mutualInfo');
end
%*****************


% add legacy code to path
addpath('sim-code');

startSimulation(qrs.config('Algorithm'),qrs.config('Strategy'));
