#!/bin/bash

mv output301.png output391.png

for i in `seq 361 390`;
do
	cp "output300.png" "output$i.png"
done

mv "output300.png" "output360.png"

for i in `seq 392 420`;
do
	cp "output391.png" "output$i.png"
done

mv "output1001.png" "output331.png"
for i in `seq 301 330`;
do
	cp "output1000.png" "output$i.png"
done
mv output1000.png output300.png

for i in `seq 332 360`;
do
	cp "output331.png" "output$i.png"
done
