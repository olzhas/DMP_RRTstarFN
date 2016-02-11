#!/bin/sh

OUTPUT_DIRECTORY="removal_state"

if [ ! -d $OUTPUT_DIRECTORY ]; then
	mkdir $OUTPUT_DIRECTORY
fi

n=800

res="dubins-colored-results-7.dat"
vert="dubins-vertices$n.dat"
edge="dubins-edges$n.dat"
fout="$OUTPUT_DIRECTORY/output.png"

gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=2; obstacle=1" tree.gnu

