#!/bin/sh

gnuplot tree.gnu

for i in `seq 0 299`;
do
	res="dubins-results-interp$i.txt"
	vert="dubins-vertices$i.dat"
	edge="dubins-edges$i.dat"
	fout="output$i.png"

	gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=1" tree.gnu
	echo "Done: $i"
done

## dynamic obstacles moves

res="dubins-results-interp299.txt"
vert="dubins-vertices299.dat"
edge="dubins-edges299.dat"
fout="output1000.png"

gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=3" tree.gnu

## previous position erased

res="dubins-results-interp299.txt"
vert="dubins-vertices299.dat"
edge="dubins-edges299.dat"
fout="output1001.png"

gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=2" tree.gnu

# ## tree clean-up
#
# res="dubins-results300.txt"
# vert="dubins-vertices300.dat"
# edge="dubins-edges300.dat"
# fout="output1002.png"
#
# gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=2" tree.gnu


for i in `seq 300 301`;
do
	res="dubins-results-interp$i.txt"
	vert="dubins-vertices$i.dat"
	edge="dubins-edges$i.dat"
	fout="output$i.png"
	gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=2; dynamic=1" tree.gnu
done


mkdir dynamic_reconnect
cd dynamic_reconnect