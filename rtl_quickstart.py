# pip install pyrtlsdr


import numpy as np
import time
import torch
import plotly.graph_objects as go

# Use either PlutoSDR or RTL-SDR based on available hardware
use_rtlsdr = True  # Set to False to use PlutoSDR instead

# Common parameters
fc = 98e6  # Center frequency
fs = 2.4e6  # Sample rate (note: RTL-SDR has max of 2.4 MSPS for stable operation)
Nsamp = 2**10
numRows = 1

if use_rtlsdr:
    from rtlsdr import RtlSdr

    # Configure RTL-SDR
    sdr = RtlSdr()
    sdr.center_freq = int(fc)
    sdr.sample_rate = int(fs)
    sdr.gain = 40.0  # RTL-SDR uses different gain values
else:
    import adi

    # Configure PlutoSDR
    sdr = adi.Pluto("ip:192.168.2.1")
    sdr.rx_lo = int(fc)
    sdr.sample_rate = int(fs)
    sdr.gain_control_mode_chan0 = 'manual'
    sdr.rx_hardwaregain_chan0 = 70.0
    sdr.rx_buffer_size = int(Nsamp)
    sdr.rx_destroy_buffer()

# Setup plot
fftSize = Nsamp
window = np.blackman(fftSize)
freq = np.fft.fftshift(np.fft.fftfreq(fftSize, 1/fs)) + fc
specData = np.zeros((numRows, fftSize))
time_axis = (np.arange(numRows) + 1) * fftSize/fs

# Timing calculation
Texpected = Nsamp/fs*numRows
print('time expected (s) ' + str(Texpected))

tic = time.time()
for i in range(numRows):
    if use_rtlsdr:
        # Get samples from RTL-SDR
        samples = sdr.read_samples(Nsamp)
    else:
        # Get samples from PlutoSDR
        samples = sdr.rx()
    
    spectrum = np.fft.fftshift(np.fft.fft(samples * window))
    powerDb = 10 * np.log10(np.abs(spectrum)**2)
    specData = np.vstack([specData[1:], powerDb])

toc = time.time() - tic
print(Texpected/toc)  # check that we aren't losing too much data

# Create and display plot
fig = go.Figure()
fig.update_layout(template='plotly_dark')
fig.add_trace(go.Scatter(x=freq, y=powerDb))
fig.show()

print(f"Total time: {time.time() - tic} seconds")

# Clean up
if use_rtlsdr:
    sdr.close()
    