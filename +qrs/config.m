function v = config(varargin)
% QRS.CONFIG retrieve global config info
%   C = QRS.CONFIG returns a handle to the containers.Map object containing
%   the global config.
%
%   V = QRS.CONFIG(K) returns the global config value for key K
%
%   See also QRS.R.CONFIGURATION

if nargin == 0
	v = qrs.configuration.getconfig;
else
	v = qrs.configuration.get(varargin{1});
end