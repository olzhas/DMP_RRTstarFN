#!/bin/sh

OUTPUT_DIRECTORY="dynamic_plots"

if [ ! -d $OUTPUT_DIRECTORY ]; then
	mkdir $OUTPUT_DIRECTORY
fi
n=300
i=10

res="dubins-colored-results-$i.dat"
vert="dubins-vertices$n.dat"
edge="dubins-edges$n.dat"
fout="$OUTPUT_DIRECTORY/pre/__output$i.png"
gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=2; obstacle=1; highlight=1" tree.gnu
