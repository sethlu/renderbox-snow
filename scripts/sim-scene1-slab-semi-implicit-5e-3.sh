#!/usr/bin/env bash

TIME_STEP=5e-3
DIRECTORY=scene1-slab-d400-semi-implicit-$TIME_STEP
FRAMES=120

rm -rf $DIRECTORY
mkdir $DIRECTORY
cd $DIRECTORY

../snow sim-gen-slab $TIME_STEP 1
../snow sim-scene1 0 $FRAMES
