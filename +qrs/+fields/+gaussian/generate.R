# generate.R seedfile outfile model N res [params ...]
#   Generate a 2D Gaussian random field of size NxN with resolution res. The
#   model and params values are passed directly to GaussRF. The resulting
#   field is stored in outfile as a MATLAB-compatible binary file.

library(R.matlab)
library(RandomFields)
source("R.qrs/rngstate.R")

# parse arguments
args <- commandArgs(trailingOnly=TRUE)
statefile  = args[1]
filename	 = args[2]
model      = args[3]
width      = as.integer(args[4])
resolution = as.numeric(args[5])
param      = as.numeric(as.vector(args[-1:-5]))

LoadRNG(statefile)

# generate field and write to matlab binary format
data <- GaussRF(x     = seq(1, width, resolution),
						    y     = seq(1, width, resolution),
							  grid  = TRUE,
						 		model = model,
						 		param = param)
writeMat(filename, data=data)

SaveRNG(statefile)