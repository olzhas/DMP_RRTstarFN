file_exists(file) = system("[ -f '".file."' ] && echo '1' || echo '0'") + 0

if (!exists("filenameEdges")) filenameEdges='dubins-edges.dat'
if (!exists("filenameVertices")) filenameVertices='dubins-vertices.dat'
if (!exists("filenameOutput")) filenameOutput='output.png'

set term pngcairo size 1920,1080
#set terminal postscript eps enhanced color font 'Helvetica,10' size 8,8
set output filenameOutput

#unset ytics
#unset xtics
set size ratio -1
set style arrow 1 linecolor rgb "#999999" linewidth 1
set style arrow 2 nohead linecolor rgb "#FFA500" linewidth 1
set style fill transparent solid 0.5 noborder

set style arrow 3 nohead linewidth 5
set style line 1 linecolor rgb "#11CC22" linewidth 4

set multiplot

set style fill solid 1.0 border -1

if (circle == 1) {
	#set object 2 circle at 900,1050 size 100 fc rgb "#FF4444" front
	#set object 4 circle at 200,1000 size 150 fc rgb "#FF4444" front
	#set object 5 circle at 500,1450 size 100 fc rgb "#FF4444" front
}
if (circle == 2) {
	
	#set object 3 circle at 1070,1400 size 100 fs solid fc rgb "#2244FF" front 
	#set object 4 circle at 200,1000 size 150 fc rgb "#FF4444" front
	#set object 5 circle at 500,1450 size 100 fc rgb "#FF4444" front
}
if (circle == 3) {
	#set object 2 circle at 900,1050 size 100 fs solid fc rgb "#FF4444" front
	#set object 3 circle at 1070,1400 size 100 fs solid fc rgb "#2244FF" front 
	#set object 4 circle at 200,1000 size 150 fc rgb "#FF4444" front
	#set object 5 circle at 500,1450 size 100 fc rgb "#FF4444" front
}

set yrange [0:2160]
set xrange [0:3840]


if (!exists("dynamic")) {
	plot filenameEdges using 1:2:($4-$1):($5-$2) notitle with vectors arrowstyle 1
} else {
	if (dynamic == 1) {
		plot filenameEdges using 1:2:($4-$1):($5-$2):($7) notitle with vectors lc rgbcolor variable lw 1 nohead
	}
}

plot filenameVertices using 1:2 notitle fc rgb "#6666DD" pointtype 5 pointsize 0.5

if (exists("highlight")) {
	plot "highlight.dat" using 1:2:($4-$1):($5-$2) notitle with vectors arrowstyle 2
}

if (exists("filenameResults") && file_exists(filenameResults)) {
	plot filenameResults using 1:2 notitle linestyle 1 pointtype 1
}

if (exists("filenameResultsMilestones") && file_exists(filenameResultsMilestones)) {
	plot filenameResultsMilestones using 1:2 notitle linestyle 1 pointtype 1
}





