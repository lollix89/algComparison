# QRS: Mobile Sensing Simulator
---

This project simulates both mobile and static sensors measuring environmental fields. Various field types and sensing strategies are available.

## Running
---

### Locally

QRS can be executed locally in MATLAB, simply by calling qrs.run and supplying simulation parameters directly.

```matlab
qrs.run('FieldModel','spherical','FieldRange',50);
```

### On the Cluster

Scripts to manage large simulations on the cluster are also provided. qrs.sub serves as a qsub script template, which accepts as parameters the configuration string to be passed to qrs.run. To run the above example 100 times on the cluster, you might use:

```bash
qsub -N "example" -t 1-100 scripts/qrs.sub "'FieldModel','spherical','FieldRange',50"
```

More likely, you're planning to try repeated tests of a few different parameter combinations. Best practices dictate that you should make a new script for such an experiment. The example below uses a couple of convenience scripts.

_qclean_ launches a job that waits for a specified job to complete, and then aggregates all of the results in a directory into a single file.

_qjob_ is synonymous with qsub, except that it only prints out the job ID. This makes it easy to pass to subsequently call qclean.

```bash
#!/bin/bash

DATA_DIR_PATTERN="./data/\$JOB_NAME"

FIELD_MODEL='spherical'
NUM_RUNS=10

JOB_NAME="qrs-short-range"
eval DATA_DIR=$DATA_DIR_PATTERN
JOB_ID=$( ./qjob -N "$JOB_NAME" -t 1-$NUM_RUNS scripts/qrs.sub \
	"'DataDirectory','$DATA_DIR'," \
	"'FieldRange',20,"             \
	"'FieldModel','$FIELD_MODEL'"  ) && \
	./qclean $JOB_ID "$DATA_DIR.txt" "$DATA_DIR" > /dev/null

JOB_NAME="qrs-medium-range"
eval DATA_DIR=$DATA_DIR_PATTERN
JOB_ID=$( ./qjob -N "$JOB_NAME" -t 1-$NUM_RUNS scripts/qrs.sub \
	"'DataDirectory','$DATA_DIR'," \
	"'FieldRange',30,"             \
	"'FieldModel','$FIELD_MODEL'"  ) && \
	./qclean $JOB_ID "$DATA_DIR.txt" "$DATA_DIR" > /dev/null

JOB_NAME="qrs-long-range"
eval DATA_DIR=$DATA_DIR_PATTERN
JOB_ID=$( ./qjob -N "$JOB_NAME" -t 1-$NUM_RUNS scripts/qrs.sub \
	"'DataDirectory','$DATA_DIR'," \
	"'FieldRange',40,"             \
	"'FieldModel','$FIELD_MODEL'"  ) && \
	./qclean $JOB_ID "$DATA_DIR.txt" "$DATA_DIR" > /dev/null
```

An example is provided as [qrs/run-example](../blob/master/run-example)

## Parameters
---

Simulation parameters are passed directly to qrs.run.

### RandomSeed

Integer. Optional. Fixes the random seed for the simulation to the specified value.

### Algorithm

String. Required. mutualInfo or kriging

### Strategy

String. Required. ACO or greedy

### FieldRange

Integer. Required. Correlation range of generated random field.
