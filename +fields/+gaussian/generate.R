library(R.matlab)
library(RandomFields)

# parse arguments
args <- commandArgs(trailingOnly=TRUE)
filename	 = args[1]
model      = args[2]
width      = as.integer(args[3])
resolution = as.numeric(args[4])
param      = as.numeric(as.vector(args[-1:-4]))

# generate field and write to matlab binary format
data <- GaussRF(x     = seq(1, width, resolution),
						    y     = seq(1, width, resolution),
							  grid  = TRUE,
						 		model = model,
						 		param = param)
writeMat(filename, data=data)