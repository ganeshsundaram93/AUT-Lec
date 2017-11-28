# Computes Spectrogram from sample glas data set (5 def, 5 ok)
# Shows Spectrograms and saves those as gray value bmp

from PIL import Image
import numpy as np
from scipy import signal 
import matplotlib.pyplot as plt
import pylab
from scikits.audiolab import Sndfile
from scipy.io import savemat

ndata= np.load('glas_2016.npy')
x, y = ndata.shape
print "Data shape:", ndata.shape

Fs=22058

pylab.plot(ndata[:,5])
pylab.show()

#plt.hist(norm_ndata[6550:60000], bins=15, color='blue')

#Lab 4 Spectrogram

for k in range(0,10):
  	NFFT = 128
  # the sampling frequency Fs
  #samples=x
	f, t, Sxx = signal.spectrogram(ndata[:,k], Fs, nperseg=128, noverlap=64, mode = 'magnitude')
	plt.pcolormesh(t, f, Sxx)
	plt.ylabel('Frequency [Hz]')
	plt.xlabel('Time [sec]')
	plt.show()

	print "Sxx shape:", Sxx.shape

	Sxx = Sxx*200             # Heuristic scaling factor
	Sxx = Sxx.astype('uint8')

	print "Sxx data type:", Sxx.dtype

	img = Image.fromarray(Sxx)
	img.save("Glas_spec"+str(k+1)+".bmp")
	img.show()

#pylab.plot(Sxx[:,400])
#pylab.show()





