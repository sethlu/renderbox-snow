#!/usr/bin/env bash

DIRECTORY=scene1-slab-d60-explicit-5e-4
FRAMES=120

rm -rf $DIRECTORY
mkdir $DIRECTORY
cd $DIRECTORY

../snow sim-gen-slab
../snow sim-scene1 0 $FRAMES
