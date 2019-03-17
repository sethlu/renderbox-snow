#!/usr/bin/env bash

TIME_STEP=5e-3
DIRECTORY=scene0-snowball-d60-semi-implicit-$TIME_STEP
FRAMES=120

rm -rf $DIRECTORY
mkdir $DIRECTORY
cd $DIRECTORY

../snow sim-gen-snowball $TIME_STEP 1
../snow sim-scene0 0 $FRAMES
