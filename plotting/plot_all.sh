#!/bin/sh

gnuplot tree.gnu

for i in `seq 0 299`;
do
	res="dubins-results$i.txt"
	vert="dubins-vertices$i.dat"
	edge="dubins-edges$i.dat"
	fout="output$i.png"

	gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=1" tree.gnu
	echo "Done: $i"
done

for i in `seq 300 301`;
do
	res="dubins-results$i.txt"
	vert="dubins-vertices$i.dat"
	edge="dubins-edges$i.dat"
	fout="output$i.png"
	gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=2" tree.gnu
done

