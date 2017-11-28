import numpy as np
from scikits.audiolab import Sndfile

data= np.load('glas_2016.npy')
#ndata = np.reshape(data,(data.shape[0],1))  # Input of whole data set in .npy
print "shape:", data.shape

import pylab 
pylab.plot(data[:,0])

pylab.show()

print "should give a plot of the sound data:"

