#!/bin/sh

OUTPUT_DIRECTORY="dynamic_plots"

if [ ! -f $OUTPUT_DIRECTORY ]; then
	mkdir $OUTPUT_DIRECTORY
fi

n=300
for i in `seq 1 9`;
do 
	res="dubins-colored-results-$i.dat"
	vert="dubins-vertices$n.dat"
	edge="dubins-edges$n.dat"
	fout="$OUTPUT_DIRECTORY/output$i.png"
	gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=2; obstacle=1" tree.gnu
done
