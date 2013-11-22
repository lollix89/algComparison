function init
% QRS.CONFIGURATION.INIT initializes global config

global QRS_CONFIG;

if isempty(QRS_CONFIG)
	QRS_CONFIG = containers.Map;
end