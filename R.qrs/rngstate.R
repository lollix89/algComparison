# Functions for managing R's RNG state. The state can be saved and loaded from
# disk, allowing for a persistent RNG across multiple invocations of R.

# save RNG state to filename
SaveRNG <- function(filename) {
  rngstate <- get(".Random.seed", .GlobalEnv)
  save("rngstate", file=filename)
}

# load RNG state from filename
LoadRNG <- function(filename) {
  load(filename)
  assign(".Random.seed", rngstate, .GlobalEnv)
}