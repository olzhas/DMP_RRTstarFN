#!/bin/sh

avconv -framerate 1 -f image2 -i 'Capture%4d.png' -c:v h264 -crf 1 -s 800x600 -r 30 out.mp4

