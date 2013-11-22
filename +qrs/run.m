function run(varargin)
% QRS.RUN execute quadrotor simulation
%   QRS.RUN(C1,V1,...,CN,VN) starts a simulation with a configuration
%   specified by the provided C = V pairs of config keys and values.

qrs.configuration.set(varargin{:});

% initialize RNGs
if isKey(qrs.config,'RandomSeed')
	rng(qrs.config('RandomSeed'));
	qrs.r.init(qrs.config('RandomSeed'));
else
	qrs.r.init;
end

randi(100000)

% TODO: run simulation here
