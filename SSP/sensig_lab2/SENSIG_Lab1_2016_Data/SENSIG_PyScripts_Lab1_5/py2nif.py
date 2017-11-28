#############################################################
# Written for ISE 
# author: Abhaya Chandra K <abhay@eit.uni-kl.de>
#############################################################

import scipy.io
import numpy as np
import string

def py2nif(data,filename):
    """
    Convert from python array to .nif  for Quickcog

    Keyword arguments:
    data: MxN numpy array 
    filename: filename of .nif output file
    """
    outfile = open(filename,"w")
    r,c = data.shape
    outfile.write(str(r)+", "+str(c)+"\n")
    outfile.write("{:18.17e}".format(np.max(data))+", "+"{:18.17e}".format(np.min(data))+"\n")
    outfile.write("\nASCII \n")
    outfile.write("\n0\n")
    for x in xrange(c):
        outfile.write("no_info\n")
        for y in xrange(r):
            outfile.write("{:18.17e}".format(data[y,x])+"\n")
    outfile.close
                          
if __name__=='__main__':
    iris = scipy.io.loadmat('IrisData.mat')['iris']
    irisclass = scipy.io.loadmat('IrisData.mat')['irisclass']
    py2nif(iris,"iris.nif")
    py2nif(irisclass,"irisclass.nif")
