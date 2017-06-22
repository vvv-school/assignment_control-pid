#! /bin/bash

rp=$(realpath $0)
ap=$(dirname ${rp})

out=${HOME}/.local/share/yarp/contexts/assignment_control-pid/out.txt
octave --persist --no-gui ${ap}/octave-script.m ${out}
