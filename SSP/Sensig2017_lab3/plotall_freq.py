import numpy as np
import scipy
import pylab

ndata= np.load("glass.npy")
print "shape 0:", ndata.shape

x,y = ndata.shape
#Fs=22058
Fs=44100

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
        #~ Y = Y[:n/2]
        Y = Y[1:n/2+1] # The first frequency is removed due to its high amplitude
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
np.save("glass_spec.npy",abs(freqdata))
plotall(abs(freqdata),y)

import py2nif as p2n
p2n.py2nif(abs(freqdata),"Glass_MagSpec.nif")


