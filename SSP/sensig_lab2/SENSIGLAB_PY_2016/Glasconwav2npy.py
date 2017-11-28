import numpy as np
from scikits.audiolab import Sndfile
from scipy.io import savemat
f = Sndfile('glassounds/Glas_def_1.wav', 'r')
print f
# Sndfile instances can be queried for the audio file meta-data
fs = f.samplerate
nc = f.channels
enc = f.encoding
print "number of samples:",fs
# Reading is straightfoward
data = f.read_frames(71669)
data = np.reshape(data,(data.shape[0],1))
print "shape of data:", data.shape
print "data values:", data
f = Sndfile('glassounds/Glas_def_2.wav', 'r')
temp = f.read_frames(71669)
temp = np.reshape(temp,(temp.shape[0],1))
data = np.concatenate((data, temp), axis=1)
#
f = Sndfile('glassounds/Glas_def_3.wav', 'r')
temp = f.read_frames(71669)
temp = np.reshape(temp,(temp.shape[0],1))
data = np.concatenate((data, temp), axis=1)
#
f = Sndfile('glassounds/Glas_def_4.wav', 'r')
temp = f.read_frames(71669)
temp = np.reshape(temp,(temp.shape[0],1))
data = np.concatenate((data, temp), axis=1)
#
f = Sndfile('glassounds/Glas_def_5.wav', 'r')
temp = f.read_frames(71669)
temp = np.reshape(temp,(temp.shape[0],1))
data = np.concatenate((data, temp), axis=1)
#
f = Sndfile('glassounds/Glas_ok_1.wav', 'r')
temp = f.read_frames(71669)
temp = np.reshape(temp,(temp.shape[0],1))
data = np.concatenate((data, temp), axis=1)
#
f = Sndfile('glassounds/Glas_ok_2.wav', 'r')
temp = f.read_frames(71669)
temp = np.reshape(temp,(temp.shape[0],1))
data = np.concatenate((data, temp), axis=1)
#
f = Sndfile('glassounds/Glas_ok_3.wav', 'r')
temp = f.read_frames(71669)
temp = np.reshape(temp,(temp.shape[0],1))
data = np.concatenate((data, temp), axis=1)
#
f = Sndfile('glassounds/Glas_ok_4.wav', 'r')
temp = f.read_frames(71669)
temp = np.reshape(temp,(temp.shape[0],1))
data = np.concatenate((data, temp), axis=1)
#
f = Sndfile('glassounds/Glas_ok_5.wav', 'r')
temp = f.read_frames(71669)
temp = np.reshape(temp,(temp.shape[0],1))
data = np.concatenate((data, temp), axis=1)
#
np.save('glas_2016.npy',data)
print "shape of data:", data.shape
print "data values:", data

import py2nif as p2n
p2n.py2nif(data,"Glas_Py.nif")
class_labels = np.array([2,2,2,2,2,1,1,1,1,1])
class_labels= np.reshape(class_labels,(1,class_labels.shape[0]))
print "class labels:", class_labels
p2n.py2nif(class_labels,"Glas_MagSpec_Class.nif")

import pylab 
pylab.subplot(2,5,1)
pylab.plot(data[:,0])
pylab.subplot(2,5,2)
pylab.plot(data[:,1])
pylab.subplot(2,5,3)
pylab.plot(data[:,2])
pylab.subplot(2,5,4)
pylab.plot(data[:,3])
pylab.subplot(2,5,5)
pylab.plot(data[:,4])
pylab.subplot(2,5,6)
pylab.plot(data[:,5])
pylab.subplot(2,5,7)
pylab.plot(data[:,6])
pylab.subplot(2,5,8)
pylab.plot(data[:,7])
pylab.subplot(2,5,9)
pylab.plot(data[:,8])
pylab.subplot(2,5,10)
pylab.plot(data[:,9])




pylab.show()

print "should give a plot of the sound data:"

