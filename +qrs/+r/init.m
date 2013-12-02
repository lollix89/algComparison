function init(varargin)
% QRS.R.INIT initialize R wrapper
%   QRS.R.INIT intializes the R wrapper, seeding the RNG to R's default value
%   and saving its state to a new temporary file.
%
%   QRS.R.INIT(SEED) seeds the RNG to a specific value specified by SEED.
%
%   See also QRS.R.EXEC

global QRS_R_RNGSTATE;
QRS_R_RNGSTATE = tempname;

if nargin == 1
	% offset seed by some value just in case they use exactly the same rng as
	% matlab (they don't)
	qrs.r.exec('R.qrs/initrng.R','%u',varargin{1}+99);
else
	qrs.r.exec('R.qrs/initrng.R');
end
