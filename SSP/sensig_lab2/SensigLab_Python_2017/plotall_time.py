import numpy as np
import sys

# validation.
if len(sys.argv) < 2:
    print "Plots all waves in time domain.\n\n" +\
          "Usage: %s filename.npy" % sys.argv[0]
    sys.exit(-1)

data= np.load(sys.argv[1])
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

