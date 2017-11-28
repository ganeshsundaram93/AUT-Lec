import numpy as np
import scipy
import pylab
from scikits.audiolab import Sndfile
from scipy.io import savemat

magspecdata= np.load('glas_2016_spec.npy')

x,y = magspecdata.shape
Fs=22058


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

for i in range (0, 5):
		if i==0: 
			Spec_C1 = magspecdata[:,0]
			Spec_C2 = magspecdata[:,5]
		else:
			Spec_C1 = Spec_C1 + magspecdata[:,i]
			Spec_C2 = Spec_C2 + magspecdata[:,i+5]

Spec_Diff = abs(Spec_C1-Spec_C2)/5
pylab.plot(Spec_Diff)
pylab.show()

for i in range (0, 10):
	magspecdata[:,i]=magspecdata[:,i]*Spec_Diff	

plotall(magspecdata,y)
np.save('glas_2016_spec_sel.npy',magspecdata)

import py2nif as p2n
p2n.py2nif(magspecdata,"Glas_MagSpec_Sel.nif")
#class_labels = np.array([2,2,2,2,2,1,1,1,1,1])
#class_labels= np.reshape(class_labels,(1,class_labels.shape[0]))
#print "class labels:", class_labels
#p2n.py2nif(class_labels,"Glas_MagSpec_Class.nif")

# Compact the data by discarding subthreshold frequency contributions

Thresh = 0.0001    # Tentative Setting
cnt = 0
for k in range (0, x):
	if (Spec_Diff[k] > Thresh): 
			cnt = cnt + 1

print "No. of valid features", cnt

magspecseldata = magspecdata[0:cnt,:]

print "Size after selection", magspecseldata.shape


cnt2 = 0
for k in range (0, x):
	if (Spec_Diff[k] > Thresh):
         	for i in range (0, 10):
			magspecseldata[cnt2,i]=magspecdata[k,i]
		cnt2 = cnt2 +1
			
print "Size after compaction", magspecseldata.shape

np.save('glas_2016_spec_sel_comp.npy', magspecseldata)
p2n.py2nif(magspecseldata,"Glas_MagSpec_Sel_Comp.nif")
