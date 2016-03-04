file_exists(file) = system("[ -f '".file."' ] && echo '1' || echo '0'") + 0

if (!exists("filenameEdges")) filenameEdges='dubins-edges.dat'
if (!exists("filenameVertices")) filenameVertices='dubins-vertices.dat'
if (!exists("filenameOutput")) filenameOutput='output.png'

set term pngcairo size 1440, 1440
#set terminal postscript eps enhanced color font 'Helvetica,10' size 8,8
set output filenameOutput

unset ytics
unset xtics
set size square
set style arrow 1 nohead linecolor rgb "#999999" linewidth 1
set style arrow 2 nohead linecolor rgb "#FFA500" linewidth 1

set style arrow 3 nohead linewidth 5
set style line 1 linecolor rgb "#11CC22" linewidth 4

set multiplot

set object 1 circle at 1.7,1.0 size .025 fc rgb "#336699" lw 5

set style fill solid 1.0 border -1

if (circle == 1) {
	#set object 2 circle at 0.8,1.0 size .1 fc rgb "#22FF4444" front
	#set object 2 circle at 0.9,1.0 size .1 fc rgb "#22FF4444" front
	set object 2 circle at 0.9,1.05 size .1 fc rgb "#22FF4444" front
	set object 4 circle at 0.2,1.0 size .1 fc rgb "#22FF4444" front
	set object 5 circle at 0.5,1.45 size .1 fc rgb "#22FF4444" front
}
if (circle == 2) {
	#set object 3 circle at 0.799238939618349,1.01743114854953 size .1 fs solid fc rgb "#222244FF" front
	#set object 3 circle at 0.9518,1.1932 size .1 fs solid fc rgb "#222244FF" front
	#
	#set object 3 circle at 0.9,1.2 size .1 fs solid fc rgb "#222244FF" front 
	#
	set object 3 circle at 1.07,1.4 size .1 fs solid fc rgb "#222244FF" front 
	set object 4 circle at 0.2,1.00 size .1 fc rgb "#22FF4444" front
	set object 5 circle at 0.5,1.45 size .1 fc rgb "#22FF4444" front
}
if (circle == 3) {
	#set object 2 circle at 0.6,1.0 size .1 fs solid fc rgb "#22FF4444" front
	#set object 3 circle at 0.799238939618349,1.01743114854953 size .1 fs solid fc rgb "#222244FF" front
	#set object 2 circle at 0.9,1.0 size .1 fs solid fc rgb "#22FF4444" front
	set object 2 circle at 0.9,1.05 size .1 fs solid fc rgb "#22FF4444" front
	set object 3 circle at 1.07,1.4 size .1 fs solid fc rgb "#222244FF" front 
	###set object 3 circle at 0.9,1.2 size .1 fs solid fc rgb "#222244FF" front 
	#set object 3 circle at 0.9518,1.1932 size .1 fs solid fc rgb "#222244FF" front #75

	set object 4 circle at 0.2,1.0 size .1 fc rgb "#22FF4444" front
	set object 5 circle at 0.5,1.45 size .1 fc rgb "#22FF4444" front

}

set xrange [0:2]
set yrange [0:2]

if (!exists("dynamic")) {
	plot filenameEdges using 1:2:($4-$1):($5-$2) notitle with vectors arrowstyle 1
} else {
	if(dynamic == 1){
		plot filenameEdges using 1:2:($4-$1):($5-$2):($7) notitle with vectors lc rgbcolor variable lw 1 nohead
	}
}

plot filenameVertices using 1:2 notitle fc rgb "#6666DD" pointtype 5 pointsize 0.5

if (exists("filenameResults")){
	if (exists("obstacle")) {
		plot filenameResults using 1:2:($4) notitle lc rgbcolor variable lw 4
	} else {
		plot filenameResults using 1:2 notitle linestyle 1 pointtype 1	
	}
}

if (exists("filenameResultsMilestones") && file_exists(filenameResultsMilestones)) {
	plot filenameResultsMilestones using 1:2 notitle lc rgb "#FFAA11" pointtype 12 pointsize 5 lw 5
}

plot 'obstacles.dat' using 1:2:($3-$1):($4-$2):($5) notitle with vectors nohead lw 10 linecolor rgbcolor variable

if (exists("highlight")) {
	plot "highlight.dat" using 1:2:($4-$1):($5-$2) notitle with vectors arrowstyle 2
}