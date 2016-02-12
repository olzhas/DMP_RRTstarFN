#!/usr/bin/python

import numpy as np

a = 300
b = 800

before = np.loadtxt('dubins-edges300.dat', usecols=(0,1,2,3,4,5))
after = np.loadtxt('dubins-edges800.dat', usecols=(0,1,2,3,4,5))

np.savetxt('reworked_dubins-edges300.dat', before, delimiter=' ', fmt='%5.6f')
np.savetxt('reworked_dubins-edges800.dat', after, delimiter=' ', fmt='%5.6f')
