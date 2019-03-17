#!/usr/bin/env bash

DIRECTORY=scene0-snowball-d60-explicit-5e-4
FRAMES=120

rm -rf $DIRECTORY
mkdir $DIRECTORY
cd $DIRECTORY

../snow sim-gen-snowball
../snow sim-scene0 0 $FRAMES
