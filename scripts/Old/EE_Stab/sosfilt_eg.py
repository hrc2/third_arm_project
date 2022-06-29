import numpy as np
import scipy.signal as sg
import math

# Transfer functions for IIR low-pass differentiators of degree two, Cut-off at 0.2 * (sample_freq/2)
b_ld = np.array([-0.0795571277, 0.1390709784, -0.0479192600, -0.0031459045, -0.0084486862])
a_ld = np.array([1, -1.571029458, 1.459212744, -0.7173743414, 0.1488005975])

# Transfer functions for IIR low-pass filters
b = np.array([0.1400982208,	-0.0343775491, 0.0454003083, 0.0099732061, 0.0008485135])
a = np.array([1, -1.9185418203,	1.5929378702, -0.5939699187, 0.0814687111])

#sosmat_lf = sg.tf2sos(b, a)
#sosmat_lpd = sg.tf2sos(b_ld, a_ld)

#x = np.array([1.17, 1.2, 1.22, 1.18, 1.16, 1.23])
N = 1000
x = np.ones((N,2)) + 0.05*np.random.rand(N,2)
sample = x[-math.ceil(N/50): -1, :]

#print(x)
#print(sample)

#x_filt_lf = sg.sosfilt(sosmat_lf, sample)
#x_filt_lpd = sg.sosfilt(sosmat_lpd, sample)
x_filt_lf = sg.filtfilt(b, a, x=sample, axis=0)
#x_filt_lpd = sg.filtfilt(b_ld, a_ld, x=sample, axis=0)

print('Original last sample:', x[-1,:])

print('Filtered last sample, IIR low-pass:', x_filt_lf[-1])
#s = x_filt_lf.shape[0]
s = np.array([])
print('Shape: ', len(s))
#print('Filtered last sample, IIR low-pass differentiator:', x[-1] + x_filt_lpd[-1])

