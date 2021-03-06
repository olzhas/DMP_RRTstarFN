#!/bin/sh

OUTPUT_DIRECTORY="dynamic_plots"

if [ ! -d $OUTPUT_DIRECTORY ]; then
	mkdir $OUTPUT_DIRECTORY
fi

n=241
for i in `seq 1 12`;
do 
	res="dubins-colored-results-$i.dat"
	vert="dubins-vertices$n.dat"
	edge="dubins-edges$n.dat"
	fout="$OUTPUT_DIRECTORY/output$i.png"
	gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=2; obstacle=1" tree.gnu
done

if [ ! -d "$OUTPUT_DIRECTORY/pre" ]; then
	mkdir -p "$OUTPUT_DIRECTORY/pre"
fi

n=800

for i in `seq 1 12`;
do 
	res="dubins-colored-results-$i.dat"
	vert="dubins-vertices$n.dat"
	edge="dubins-edges$n.dat"
	fout="$OUTPUT_DIRECTORY/pre/output$i.png"
	gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=2; obstacle=1" tree.gnu
done
