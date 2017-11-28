from scipy.io import wavfile
import numpy as np
import glob
import os

nframes = 71669

data = []
labels = []
for infile in glob.glob( os.path.join("glass_2016", '*.wav') ):
    # data
    print "Reading:     ", infile
    sample_rate, wave_data = wavfile.read(infile)
    print "Sample rate: ", sample_rate
    print "Total frames:", len(wave_data)
    print "---\n"
    temp = wave_data.reshape((-1,1)) # -1 keeps the same dimension shape
    temp = temp[:nframes]
    data = temp if len(data) == 0 else np.concatenate((data, temp), axis=1)
    
    # labels
    if "ok" in infile:
        temp = np.array([1]).reshape((1,1))
        labels = temp if len(labels) == 0 else np.concatenate((labels, temp), axis=1)
    elif "def" in infile:
        temp = np.array([2]).reshape((1,1))
        labels = temp if len(labels) == 0 else np.concatenate((labels, temp), axis=1)
    else:
        print "Filename has no label ('ok', 'def')"

np.save("glass.npy", data)
np.save("glass_labels.npy", np.array(labels))
