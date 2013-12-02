function v = get(key)
% QRS.CONFIGURATION.GET(k) gets config value for key k

init;

global QRS_CONFIG;
v = QRS_CONFIG(key);