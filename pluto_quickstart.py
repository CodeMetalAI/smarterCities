# https://pysdr.org/content/pluto.html
# https://wiki.analog.com/university/tools/pluto
# https://www.youtube.com/watch?v=ph0Kv4SgSuI # dual receive X dual transmit

# XC7Z010-1CLG225C4334
# https://github.com/analogdevicesinc/hdl/tree/main/projects/pluto

# on mac, connecting is a little different: 
# ls -l /dev/tty.* 
# crw-rw-rw-  1 root  wheel   17,   2 Nov  7 15:28 /dev/tty.usbmodem1414
# screen /dev/tty.usbmodem1414 115200


# XC7Z010-1CLG225

# https://wiki.analog.com/university/tools/pluto/hacking/hardware

# fw_setenv compatible ad9364
# reboot

# iio_info -s


# sdr2use = 'pluto' # 'pluto' or 'rtl'

# import matplotlib.pyplot as plt
import numpy as np
import time
import adi
import plotly.graph_objects as go

sdr = adi.Pluto("ip:192.168.2.1") 
# use "iio_info -s" to figure out what address the pluto has
# sdr = adi.Pluto("usb:1.4.5") 
# ^^^ sometimes it's easier to use a usb address 



# pip install pylibiio

fc = 98e6
fs = 5e6
# Tint = 2 # buffer time, given there might be drops inbetween buffers
# Nsamp = int(Tint*fs)
Nsamp = 2**10
numRows = 1

# Configure SDR
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

# fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
Texpected = Nsamp/fs*numRows
print('time expected (s) ' + str(Texpected))
tic = time.time()
for i in range(numRows):
   samples = sdr.rx()
   spectrum = np.fft.fftshift(np.fft.fft(samples * window))
   powerDb = 10 * np.log10(np.abs(spectrum)**2)
   
   specData = np.vstack([specData[1:], powerDb])
toc = time.time() - tic


print(Texpected/toc) # check that we aren't loosing that much data 

fig = go.Figure()
# Nds = int(round(len(powerDb)/10e3))
fig.update_layout(template='plotly_dark')
fig.add_trace(go.Scatter(x = freq,y = powerDb)) # [::Nds]
fig.show()
print(f"Total time: {time.time() - tic} seconds")

