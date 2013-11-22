function set(varargin)
% QRS.CONFIGURATION.SET(k1,v1,...,kn,vn) sets config value k = v for all pairs

assert(mod(nargin,2) == 0,'Expected even number of arguments');

init;

global QRS_CONFIG;
for i=1:2:nargin
	QRS_CONFIG(varargin{i}) = varargin{i+1};
end
