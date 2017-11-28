import numpy as np
from scikits.audiolab import Sndfile

data= np.load('glas_2016.npy')
print "shape:", data.shape

x,y = data.shape

print "Dim:", x

vert = y/5
hor = y/2

import pylab 
for i in range (0, hor):
   for j in range (0, vert):
      pylab.subplot(vert, hor, i+j*hor+1)
      pylab.plot(data[:,i+j*hor])


pylab.show()

print "should give a plot of the sound data:"

