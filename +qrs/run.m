function run(varargin)
% QRS.RUN execute quadrotor simulation
%   QRS.RUN(C1,V1,...,CN,VN) starts a simulation with a configuration
%   specified by the provided C = V pairs of config keys and values.

clearvars -global QRS_CONFIG
qrs.configuration.set(varargin{:});

% initialize RNDs
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
if ~isKey(qrs.config,'Function')
    qrs.configuration.set('Function','linear');
end
if ~isKey(qrs.config,'Algorithm')
    qrs.configuration.set('Algorithm','mutualInfo');
end
if ~isKey(qrs.config,'Strategy')
    qrs.configuration.set('Strategy','sample');
end
if ~isKey(qrs.config,'Size')
    qrs.configuration.set('Size',300);
end
if ~isKey(qrs.config,'Estimation')
    qrs.configuration.set('Estimation',1);
end

%Parameters used only if Estimation is false i.e if the model is
%supposed known or we are running some tests to understand the effect of
%having a wrong underlkying model assumption.

if isequal(qrs.config('Estimation'), 0) && (~isKey(qrs.config,'Sill') || ~isKey(qrs.config,'Range'))
    error('[error]: If estimation function is turned off, the user must provide the parameters of sill and range!!!!!')
end

%*****************

% add code to path
addpath('sim-code');

startSimulation(qrs.config('Algorithm'),qrs.config('Strategy'));
