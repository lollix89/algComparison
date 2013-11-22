function exec(filename,varargin)
% QRS.R.EXEC execute R script
%   QRS.R.EXEC(FILENAME) executes the R script FILENAME, passing the filename
%   of the RNG state as the first and only parameter.
%
%   QRS.R.EXEC(FILENAME,ARGFORMAT,...) additionally passes the arguments
%   described in ARGFORMAT as the second parameter and onwards.
%
%   See also QRS.R.INIT

global QRS_R_RNGSTATE;
assert(~isempty(QRS_R_RNGSTATE),'qrs.r.init must be called first')

cmdstring = sprintf('Rscript --vanilla %s %s',filename,QRS_R_RNGSTATE);
if nargin > 1
	cmdstring = sprintf(['%s ' varargin{1}],cmdstring,varargin{2:end});
end

[status,cmdout] = system(cmdstring);
assert(status == 0,'R returned error code: %d\n%s',status,cmdout);