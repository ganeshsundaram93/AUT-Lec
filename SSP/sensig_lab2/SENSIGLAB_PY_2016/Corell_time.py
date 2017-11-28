import numpy as np
import scipy
import pylab
from scikits.audiolab import Sndfile
from scipy.io import savemat

ndata= np.load('glas_2016.npy')
ndata2=ndata
x, y = ndata.shape
print "Data shape:", ndata.shape

x,y = ndata.shape
Fs=22058

pylab.plot(ndata[:,0])
pylab.show()

template = ndata[22362:26400,0]
pylab.plot(template)
pylab.show()



xt= len(template)
print "Steps:",xt, x, (x-xt)

for k in range(0,10):
  Corr = ndata[0:(x-xt),k]
  for i in range(0, (x-xt)):
       Corr[i] = sum(ndata[i:i+xt,k]*template)/xt
    
  #    Temp = ndata[i:i+xt,k]*template
  #    for j in range(0, xt):
  #		if j==0: 
  #		        Temp2	=  Temp[j]
  #		else:
  #			Temp2 = Temp2 + Temp[j]
  #   Corr[i] = Temp2/xt

  Corr = abs(Corr)
  pylab.plot(Corr)
  pylab.show()

  pos = 0;
  CorrMax=0
  for i in range(0, (x-xt)): 
	if(Corr[i] > CorrMax): 
                  CorrMax = Corr[i]
                  pos = i


  print "Max pos:",pos

  pylab.plot(ndata[pos:(pos+2*xt),k])

  pylab.show()



