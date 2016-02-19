#!/usr/bin/python

import numpy as np

n = 239

filename = 'dubins-results-interp' + str(n) + '.txt'
filenameRAW = 'dubins-results' + str(n) + '.txt'

path = np.fromfile(filename, dtype=np.float64, sep=' ')
path = path.reshape(path.size/3, 3)

pathVia = np.fromfile(filenameRAW, dtype=np.float64, sep=' ')
pathVia = pathVia.reshape(pathVia.size/3, 3)

for i in xrange(0, pathVia.size/3):
	filename = 'dubins-colored-results-' + str(pathVia.size/3 - i) + '.dat'
	output = open(filename, 'w')
	flag = 0
	for j in xrange(0,path.size/3):
		test = 0
		for k in xrange(0,3):
			output.write(str(path[j, k]) + ' ')

			if(flag == 0 and path[j, k] == pathVia[i,k]):
				test = test + 1

			if(i < pathVia.size/3):
				if(flag == 1 and path[j, k] == pathVia[i+1,k]):
					test = test + 1

		if (test == 3 and flag == 1):
			flag = 0
			test = 0

		if (test == 3 and flag == 0):
			flag = 1;
			test = 0;

		if (flag == 0):
			output.write(' 0x11CC22') # greenish
		else:
			output.write(' 0xEE44EE') # purple
		output.write('\n')
	output.write('\n')
