import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
from scipy.fftpack import fft
import socket

UDP_IP = "192.168.0.163"
UDP_PORT = 3333


NUM_PIXELS = 9
B_PER_PIX  = 4
send_buff = np.zeros(NUM_PIXELS*B_PER_PIX, dtype=np.uint8)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
# Constants
SAMPLE_RATE = 44100  # Sample rate (44.1kHz)
CHUNK_SIZE = 2000    # Number of audio samples per chunk

# Set up plot for real-time visualization
plt.ion()
fig, ax = plt.subplots()
x_data = np.fft.fftfreq(CHUNK_SIZE, 1.0 / SAMPLE_RATE)[:CHUNK_SIZE // 2]
line, = ax.plot(x_data, np.zeros(CHUNK_SIZE // 2))
ax.set_xlim(20, SAMPLE_RATE // 4)  # Focus on human audible range
ax.set_ylim(0, 10)  # Adjust this based on expected magnitude range
ax.set_xlabel('Frequency (Hz)')
ax.set_ylabel('Magnitude')
plt.title("Live FFT Spectrum")

# Global buffer to store current audio chunk
buffer = np.zeros(CHUNK_SIZE)

# Audio callback function to capture audio in real-time
def audio_callback(indata, frames, time, status):
    global buffer
    if status:
        print(status)
    
    # Store audio data in buffer
    buffer = np.mean(indata, axis=1)  # Convert stereo to mono (if needed)

# Main processing loop
def update_plot():
    while True:
        # Compute FFT
        fft_data = np.abs(fft(buffer)[:CHUNK_SIZE // 2])
        
        set(fft_data[18])
        
        # Update plot
        line.set_ydata(fft_data)
        fig.canvas.draw()
        fig.canvas.flush_events()
def set(num):
    num = int(num)
    if num > NUM_PIXELS:
        num = NUM_PIXELS
    send_buff = np.zeros(NUM_PIXELS*B_PER_PIX, dtype=np.uint8)

    for i in range(num-1):
        send_buff[i*4+3] = 100
        # [G1,R1,B1,W1,....]
       # print(buffer)
    sock.sendto(send_buff, (UDP_IP, UDP_PORT))
        
# Start audio stream
with sd.InputStream(callback=audio_callback, channels=1, samplerate=SAMPLE_RATE, blocksize=CHUNK_SIZE):
    print("Streaming live audio... Press Ctrl+C to stop.")
    try:
        update_plot()  # Keep updating the plot with new FFT data
    except KeyboardInterrupt:
        print("Stopping stream.")
