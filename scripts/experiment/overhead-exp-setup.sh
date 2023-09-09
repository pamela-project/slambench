#!/bin/bash

docker build -t slambench/main .

python3 starter.py dataset -t make -d  ./datasets/KITTI/2011_09_30_drive_0027/2011_09_30_drive_0027_sync.slam -v KITTI07

python3 starter.py dataset -t make -d  ./datasets/NewerCollege/newer_college_short_quad_mid.slam -v NewerCollegeQM

python3 starter.py dataset -t make -d  ./datasets/NSH/nsh_indoor_and_outdoor.slam -v NSH

echo "y" | ./scripts/algorithm-vol.sh aloam

echo "y" | ./scripts/algorithm-vol.sh legoloam
