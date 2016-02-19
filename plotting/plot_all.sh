#!/bin/sh

#gnuplot tree.gnu

for i in `seq 0 239`;
do
	res="dubins-results-interp$i.txt"
	vert="dubins-vertices$i.dat"
	edge="dubins-edges$i.dat"
	fout="output$i.png"

	gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res';  filenameOutput='$fout'; circle=1" tree.gnu
	echo "Done: $i"
done

## dynamic obstacles moves

res="dubins-results-interp239.txt"
vert="dubins-vertices239.dat"
edge="dubins-edges239.dat"
fout="output1000.png"

gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=3" tree.gnu

## previous position erased

res="dubins-results-interp239.txt"
vert="dubins-vertices239.dat"
edge="dubins-edges239.dat"
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


for i in `seq 241 241`;
do
	res="dubins-results-interp$i.txt"
	vert="dubins-vertices$i.dat"
	edge="dubins-edges$i.dat"
	fout="output$i.png"
	gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=2; dynamic=1" tree.gnu
done

# if [ ! -d "dynamic_reconnect" ]; then
# 	echo "does not exist"
# 	mkdir "dynamic_reconnect"
# fi
