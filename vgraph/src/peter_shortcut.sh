#!/usr/bin/env bash

cd
git pull
mv vgraph/src/grow_obstacles.py catkin_ws/src/vgraph/src/grow_obstacles.py
cd catkin_ws/src/vgraph/src/
python grow_obstacles.py
