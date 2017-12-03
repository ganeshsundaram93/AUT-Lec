import numpy as np

ndata= np.load('glass.npy')
print "shape 0:", ndata.shape

x,y = ndata.shape

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

ndatamean = np.mean(ndata)
norm_ndata= ndata - ndatamean
print "shape norm:", norm_ndata.shape
plotall(norm_ndata,y)

norm_ndata_abs = np.absolute(norm_ndata)
print "shape abs:", norm_ndata_abs.shape
plotall(norm_ndata_abs,y)


#ndata_thresh = norm_ndata_abs[norm_ndata_abs>0.1]
#print "shape:", ndata_thresh.shape
#plotall(ndata_thresh,y)


