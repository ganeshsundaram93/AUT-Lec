import numpy as np
import scipy
import pylab
from scikits.audiolab import Sndfile
from scipy.io import savemat
import py2nif as p2n
import matplotlib.pyplot as plt

magspecdata= np.load('glas_2016_spec.npy')

x,y = magspecdata.shape
Fs=22058



for i in range (0, 5):
		if i==0: 
			Spec_C1 = magspecdata[:,0]
			Spec_C2 = magspecdata[:,5]
		else:
			Spec_C1 = Spec_C1 + magspecdata[:,i]
			Spec_C2 = Spec_C2 + magspecdata[:,i+5]

Spec_Diff = abs(Spec_C1-Spec_C2)/5
#pylab.plot(Spec_Diff)
#pylab.show()

for i in range (0, 10):
	magspecdata[:,i]=magspecdata[:,i]*Spec_Diff	

#plotall(magspecdata,y)
np.save('glas_2016_spec_sel.npy',magspecdata)


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
p2n.py2nif(magspecseldata,"Glas_MagSpec_Sel_Compr2.nif")

np.save('glas_2016_spec_sel_comp.npy', magspecseldata)
p2n.py2nif(magspecseldata,"Glas_MagSpec_Sel_Comp.nif")

CENTROID = np.mean(magspecseldata,axis=1)
CENTROID_C1 = np.mean(magspecseldata[:,0:5],axis=1)
CENTROID_C2 = np.mean(magspecseldata[:,5:10],axis=1)

#print"Zentroid", CENTROID.shape

SW1 = np.zeros((83,83))
SW2 = SW1

for i in range (0, 5):
    SW1 = SW1 + np.outer((magspecseldata[:,i]-CENTROID_C1),(magspecseldata[:,i]-CENTROID_C1))

for i in range (5, 10):
    SW2 = SW2 + np.outer((magspecseldata[:,i]-CENTROID_C2),(magspecseldata[:,i]-CENTROID_C2))

SW = (SW1 + SW2)/10

#print"Within", SW.shape

SB = (np.outer((CENTROID_C1-CENTROID),(CENTROID_C1-CENTROID)) + np.outer((CENTROID_C2-CENTROID),(CENTROID_C2-CENTROID)))/2

#print"Between", SB.shape

SALL = np.linalg.inv(SW)*SB

#print"Within-1*Between", SALL.shape

w,v = np.linalg.eig(SALL)

#print"Eigval(LDA)", abs(w), v.shape

magspecsellda2ddata = magspecseldata[0:2,:]

for i in range (0, 10):
	magspecsellda2ddata[0:2,i] = np.transpose(v[:,0:2]).dot(magspecseldata[:,i])
        #print"Test", magspecselpca2ddata.shape

p2n.py2nif(magspecsellda2ddata,"Glas_MagSpec_Sel_LDA.nif")
T = abs(magspecsellda2ddata)
plt.plot(T[0,0:5],T[1,0:5],'ro')
plt.plot(T[0,5:10],T[1,5:10],'bo')
plt.show()



