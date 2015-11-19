#!/usr/bin/python
import numpy as np
import math

#for x in range(1,7):
    #print np.random.uniform(0,2*math.pi, size=3)

#print '{',
#for x in range(1,7):
#    state = np.random.uniform(0.1,0.5, size=3)
#    print '{',
#    for i in range(0,3):
#        print state[i],
#        if(i < 2):
#            print ', ',
#    print '}',
#    if (x < 6):
#        print ','
#print '}'

print '{',
for x in range(1,7):
    state = np.random.uniform(-2.0,2.0, size=3)
    state_z = np.random.uniform(0.9,2, size=1)
    state[2] = (state_z)
    print '{',
    for i in range(0,3):
        print state[i],
        if(i < 2):
            print ', ',
    print '}',
    if (x < 6):
        print ','
print '}'
