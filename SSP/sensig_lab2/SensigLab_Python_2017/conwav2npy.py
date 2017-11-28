from scipy.io import wavfile
import numpy as np

sample_rate, wave_data = wavfile.read("2_euro_1.wav")
wave_data = wave_data.reshape((-1,1)) # -1 keeps the same dimension shape
np.save("2_euro_1.npy", wave_data)