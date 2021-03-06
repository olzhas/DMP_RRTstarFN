#!/bin/sh

#gnuplot tree.gnu

i=239
res="dubins-results-interp$i.txt"
resOrig="dubins-results$i.txt"
vert="dubins-vertices$i.dat"
edge="dubins-edges$i.dat"
fout="output$i.png"

echo "set style fill transparent solid 0.5 noborder" > mytree.gnu
cat ../obstacles.gnu >> mytree.gnu
echo "set style fill transparent solid 0.1 noborder" >> mytree.gnu
cat ../goal_start.txt >> mytree.gnu
cat ../dynamic_before.gnu >> mytree.gnu
cat tree.gnu >> mytree.gnu

gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameResultsMilestones='$resOrig'; filenameOutput='$fout';" mytree.gnu
echo "Done: $i"


fout="output$ia.png"

echo "set style fill transparent solid 0.5 noborder" > newtree.gnu
cat ../obstacles.gnu >> newtree.gnu
cat ../goal_start.txt >> newtree.gnu
cat ../dynamic_after.gnu >> newtree.gnu
cat tree.gnu >> newtree.gnu

gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameResultsMilestones='$resOrig'; filenameOutput='$fout'; " newtree.gnu
echo "Done: $i"



## dynamic obstacles moves

res="dubins-results-interp239.txt"
vert="dubins-vertices239.dat"
edge="dubins-edges239.dat"
fout="output1000.png"

gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameOutput='$fout'; circle=3" mytree.gnu

## previous position erased

res="dubins-results-interp239.txt"
resOrig="dubins-results239.txt"
vert="dubins-vertices239.dat"
edge="dubins-edges239.dat"
fout="output1001.png"

gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameResultsMilestones='$resOrig'; filenameOutput='$fout';" mytree.gnu

##

res="dubins-results-interp800.txt"
resOrig="dubins-results800.txt"
vert="dubins-vertices800.dat"
edge="dubins-edges800.dat"
fout="output800.png"

gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameOutput='$fout'; circle=2" mytree.gnu

##

#res="dubins-results-interp800.txt"
vert="dubins-vertices801.dat"
edge="dubins-edges801.dat"
fout="output801.png"

gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameOutput='$fout'; circle=2" mytree.gnu

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
	resOrig="dubins-results$i.txt"
	vert="dubins-vertices$i.dat"
	edge="dubins-edges$i.dat"
	fout="output$i.png"
	gnuplot -e "filenameEdges='$edge'; filenameVertices='$vert'; filenameResults='$res'; filenameResultsMilestones='$resOrig'; filenameOutput='$fout'; dynamic=1" newtree.gnu
done

# if [ ! -d "dynamic_reconnect" ]; then
# 	echo "does not exist"
# 	mkdir "dynamic_reconnect"
# fi
