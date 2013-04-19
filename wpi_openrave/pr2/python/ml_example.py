#!/usr/bin/env python

#print __doc__
#help('modules')

import datetime
import numpy as np
import pylab as pl
from matplotlib.colors import ListedColormap
from sklearn.neighbors import NearestNeighbors

#---------------------------------------------
#---------------------------------------------
now1 = datetime.datetime.now()

samples = 100*np.random.rand(10000,4)
#tmp = samples[0:10000,0:2]; samples = tmp;
#test_vector_1 = [[0, 0]]
#test_vector_2 = [[10, 10]]
test_vector_1 = [[0, 0, 0, 0]]
test_vector_2 = [[10, 10, 10, 10]]

neigh = NearestNeighbors( 10, 1, 'kd_tree' ); neigh.fit(samples)
brute = NearestNeighbors( 10, 1, 'brute' ); brute.fit(samples)

now2 = datetime.datetime.now()
print "Time to generate points : " + str(now2.second - now1.second + float(now2.microsecond - now1.microsecond)/1000000) + " secondes"

#---------------------------------------------
#---------------------------------------------
print "close to : " + str(test_vector_1)

keys = neigh.kneighbors(test_vector_1, 10, return_distance=False)

for k in keys :
    print samples[k]

print "close to : " + str(test_vector_2)

keys = neigh.kneighbors(test_vector_2, 10, return_distance=False)

for k in keys :
    print samples[k]

now3 = datetime.datetime.now()
print "Time to find points : " + str(now3.second - now2.second + float(now3.microsecond - now2.microsecond)/1000000) + " secondes"

#---------------------------------------------
#---------------------------------------------
keys = brute.kneighbors(test_vector_1, 10, return_distance=False)

print "close to : " + str(test_vector_1)

for k in keys :
    print samples[k]

print "close to : " + str(test_vector_2)

keys = brute.kneighbors(test_vector_2, 10, return_distance=False)

for k in keys :
    print samples[k]

now4 = datetime.datetime.now()
print "Time to find points : " + str(now4.second - now3.second + float(now4.microsecond - now3.microsecond)/1000000) + " secondes"
