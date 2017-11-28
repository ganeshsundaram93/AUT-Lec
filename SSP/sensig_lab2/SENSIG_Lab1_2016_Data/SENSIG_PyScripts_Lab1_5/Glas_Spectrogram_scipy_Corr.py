# Bases on previous correlation step and centered data
# Computes Spectrogram from centered sample glas data set (5 def, 5 ok)
# Shows Spectrograms and saves those as gray value bmp

from PIL import Image
import numpy as np
from scipy import signal 
import matplotlib.pyplot as plt
import pylab
from scikits.audiolab import Sndfile
from scipy.io import savemat

auxdata= np.load('glas_2016.npy')
x, y = auxdata.shape
print "Data shape:", auxdata.shape

LENGTH = 2*4038
ndata = np.zeros((LENGTH,y),dtype=float)
print "Data shape:", ndata.shape

ndata[:,0] = auxdata[22362:22362+LENGTH,0]
ndata[:,1] = auxdata[24715:24715+LENGTH,1]
ndata[:,2] = auxdata[21157:21157+LENGTH,2]
ndata[:,3] = auxdata[25864:25864+LENGTH,3]
ndata[:,4] = auxdata[23154:23154+LENGTH,4]
ndata[:,5] = auxdata[21108:21108+LENGTH,5]
ndata[:,6] = auxdata[11243:11243+LENGTH,6]
ndata[:,7] = auxdata[8920:8920+LENGTH,7]
ndata[:,8] = auxdata[16725:16725+LENGTH,8]
ndata[:,9] = auxdata[12312:12312+LENGTH,9]

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
	if (k==0):
		Mean_def = Sxx
	else:
		 if (k < 5):
			Mean_def = Mean_def+Sxx
		 else: 
			if (k==5): 
				Mean_ok = Sxx
			else : Mean_ok = Mean_ok + Sxx

	Sxxbmp = Sxx.astype('uint8')

	print "Sxx data type:", Sxxbmp.dtype

	img = Image.fromarray(Sxxbmp)
	img.save("Glas_spec_corr"+str(k+1)+".bmp")
	img.show()

Diff_Spectrogram = (abs(Mean_def-Mean_ok)/5)
plt.pcolormesh(t, f, Diff_Spectrogram)
plt.ylabel('Frequency of Diff. [Hz]')
plt.xlabel('Time [sec]')
plt.show()

x, y = Diff_Spectrogram.shape
Diff_Spectrogram_Thresh = Diff_Spectrogram
for i in range(0,x):
	for j in range(0,y):
		 if (Diff_Spectrogram[i,j] < 170): Diff_Spectrogram_Thresh[i,j] = 0

plt.pcolormesh(t, f, Diff_Spectrogram_Thresh)
plt.ylabel('Frequency of Diff. Thresh. [Hz]')
plt.xlabel('Time [sec]')
plt.show()


# Find an appropriate, i.e., well separating, threshold value and extract the remaing coefficients from the spectrograms as features !



