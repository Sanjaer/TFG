#!/bin/bash

g++ ../src/dglove.cpp -lfglove -o ../../../devel/lib/tfg/dglove
cd ../../../ && catkin_make
sleep 3
