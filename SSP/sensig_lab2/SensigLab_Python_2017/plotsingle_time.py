import numpy as np
import pylab
import sys

# validation.
if len(sys.argv) < 2:
    print "Plots single wave in time domain.\n\n" +\
          "Usage: %s filename.npy" % sys.argv[0]
    sys.exit(-1)

data= np.load(sys.argv[1])
print "shape:", data.shape
 
pylab.plot(data[:,0])

pylab.show()

print "should give a plot of the sound data"

