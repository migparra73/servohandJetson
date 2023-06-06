import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal
import sys


array = np.loadtxt(r'record.csv')

lastIdx = array[-1]
arrayResized = (signal.savgol_filter(array / ((2**32) -1 ), 201, 5))

arrayResized2 = arrayResized[0:10000]

ratio = len(arrayResized2) / len(arrayResized)

time = np.linspace(0, 10*ratio,len(arrayResized2))

arrayResized2Amp = (arrayResized2)
plt.plot(time, arrayResized2Amp)
plt.title("Finger response")
plt.xlabel("Seconds")
plt.ylabel("Volts")
plt.show()