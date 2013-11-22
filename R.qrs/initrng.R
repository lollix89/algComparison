# rngstate.R seedfile [seed]
#   Initialize R's RNG with the optional seed parameter, or to its default
#   state if not present. Save the state to seedfile.

source("R.qrs/rngstate.R")

args <- commandArgs(trailingOnly=TRUE)
statefile = args[1]
if (length(args) > 1) {
	seed = args[2]
} else {
	seed = NULL
}

set.seed(seed)
SaveRNG(statefile)
