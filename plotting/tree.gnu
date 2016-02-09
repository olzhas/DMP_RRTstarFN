if (!exists("filenameEdges")) filenameEdges='dubins-edges.dat'
if (!exists("filenameVertices")) filenameVertices='dubins-vertices.dat'
if (!exists("filenameResults")) filenameResults='dubins-results.txt'
if (!exists("filenameOutput")) filenameOutput='output.eps'

set term pngcairo size 1440, 1440
#set terminal postscript eps enhanced color font 'Helvetica,10' size 8,8
set output filenameOutput

unset ytics
unset xtics
set size square
set style arrow 1 nohead linecolor rgb "#999999" linewidth 1
set style arrow 3 nohead linewidth 5
set style line 1 linecolor rgb "#AA2222" linewidth 4

set multiplot

set object 1 circle at 1.7,1.0 size .025 fc rgb "#336699"

set style fill solid 1.0 border -1

if (circle == 1) {
	set object 2 circle at 0.6,1.0 size .1 fc rgb "#FF4444"
} 
if (circle == 2) {
	set object 2 circle at 0.799238939618349,1.01743114854953 size .1 fc rgb "#2244FF"
}

if (circle == 3) {
	set object 2 circle at 0.6,1.0 size .1 fc rgb "#FF4444"	
	set object 2 circle at 0.799238939618349,1.01743114854953 size .1 fc rgb "#2244FF"
}

set xrange [0:2]
set yrange [0:2]

plot filenameEdges using 1:2:($4-$1):($5-$2) notitle with vectors arrowstyle 1
plot filenameVertices using 1:2 notitle fc rgb "#6666DD" pointtype 5 pointsize 0.5
plot filenameResults using 1:2 notitle linestyle 1 pointtype 1
plot 'obstacles.dat' using 1:2:($3-$1):($4-$2):($5) notitle with vectors nohead lw 10 linecolor rgbcolor variable 
