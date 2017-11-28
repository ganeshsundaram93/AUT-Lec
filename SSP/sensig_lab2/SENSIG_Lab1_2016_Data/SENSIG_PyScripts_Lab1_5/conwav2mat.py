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
data = f.read_frames(88064)
dat ={'snd':data}
savemat('glas_2016.mat',dat)
print "shape of data:", data.shape
print "data values:", data

