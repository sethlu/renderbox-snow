#!/usr/bin/env bash

TIME_STEP=5e-4
DIRECTORY=scene1-slab-d60-explicit-$TIME_STEP
FRAMES=120

rm -rf $DIRECTORY
mkdir $DIRECTORY
cd $DIRECTORY

../snow sim-gen-slab $TIME_STEPs
../snow sim-scene1 0 $FRAMES
