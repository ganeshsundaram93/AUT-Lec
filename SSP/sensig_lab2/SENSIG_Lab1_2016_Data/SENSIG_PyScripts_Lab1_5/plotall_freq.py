import numpy as np
import scipy
import pylab
from scikits.audiolab import Sndfile
from scipy.io import savemat

ndata= np.load('glas_2016.npy')
print "shape 0:", ndata.shape

x,y = ndata.shape
Fs=22058

def CalcSpectrum(timedat,Fs):
    """
    Plots a Single-Sided Amplitude Spectrum of y(t)
    """
    n, sa = timedat.shape # length of the signal
    k = scipy.arange(n)
    T = n/Fs
    frq = k/T # two sides frequency range
    frq = frq[range(n/2)] # one side frequency range
    for i in range (0, sa):
    	Y = scipy.fft(timedat[:,i])/n # fft computing and normalization
    	Y = Y[range(n/2)]
	if i==0: 
		returndata = np.reshape(Y,(Y.shape[0],1))
		#print "shape0:",Y.shape
                #print "shape1:", returndata.shape
    	else: 
		Y = np.reshape(Y,(Y.shape[0],1))
		returndata=np.concatenate((returndata, Y), axis=1)
        	#print "shape2:", returndata.shape

    return returndata
   


def plotall(ydata,y):

	#print "Dim:", x
	vert = y/5
	hor = y/2
	import pylab 
	for i in range (0, hor):
   		for j in range (0, vert):
      			pylab.subplot(vert, hor, i+j*hor+1)
      			pylab.plot(ydata[:,i+j*hor])
	pylab.show()
	print "Plot of the preprocessed sound data:"


plotall(ndata,y)
freqdata= CalcSpectrum(ndata,Fs)
print "shapef:", freqdata.shape
np.save('glas_2016_spec.npy',abs(freqdata))
plotall(abs(freqdata),y)

import py2nif as p2n
p2n.py2nif(abs(freqdata),"Glas_MagSpec.nif")
class_labels = np.array([2,2,2,2,2,1,1,1,1,1])
class_labels= np.reshape(class_labels,(1,class_labels.shape[0]))
print "class labels:", class_labels
p2n.py2nif(class_labels,"Glas_MagSpec_Class.nif")


