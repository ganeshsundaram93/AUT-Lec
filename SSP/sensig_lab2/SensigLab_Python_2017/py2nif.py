#############################################################
# Written for ISE 
# author: Abhaya Chandra K <abhay@eit.uni-kl.de>
#############################################################

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
    import sys

    # validation
    if len(sys.argv) < 2:
        print "Converts Numpy file (.npy) into .nif for QuickCog.\n\n" +\
          "Usage: %s filename.npy" % sys.argv[0]
        sys.exit(-1)
    wave_data = np.load(sys.argv[1])
    py2nif(wave_data, sys.argv[1].replace("npy", "nif"))
    
