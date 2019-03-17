#!/usr/bin/env bash

DIRECTORY=scene0-snowball-explicit
FRAMES=120

rm -rf $DIRECTORY
mkdir $DIRECTORY
cd $DIRECTORY

../snow sim-init-snowball
../snow sim-scene0 frame-0.snowstate 0 $FRAMES
