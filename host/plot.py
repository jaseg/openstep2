#!/usr/bin/env python3

import serial
from matplotlib import pyplot as plt
import math
import time
import numpy as np

w = 400

def plot(chunk):
    global ydata, line
    ydata = np.concatenate((ydata[len(chunk):], chunk))
    for n, line in enumerate(lines):
        line.set_ydata(ydata[:,n])
    fig.canvas.update()
    fig.canvas.flush_events()

def chunked(gen, n):
    while True:
        l = []
        for _ in range(n):
            l.append(next(gen))
        yield l

plt.ion()

ser = serial.Serial('/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0', 500000)
time.sleep(0.5)
ser.flush()
ser.readline()
n = (len(ser.readline())-1)//3
ydata = np.zeros((w,n))

plx = math.ceil(math.sqrt(n))
ply = math.ceil(n/plx)
fig, axs = plt.subplots(plx, ply, sharex=True, sharey=True)

lines = []

for ch, ax in zip(range(n), axs.flatten()):
    ax.set_xlim([0, w-1])
    ax.set_ylim([-2048, 2047])
    line, = ax.plot(np.arange(0, w), np.zeros((w, 1)), alpha=0.7)
    lines.append(line)

def rxgen():
    while True:
        line = ser.readline()[:-1]
        if len(line) == 3*n:
            yield [int(line[i:i+3], 16)-2048 for i in range(0, 32*3, 3)]

for i, chunk in enumerate(chunked(rxgen(), 10)):
    plot(chunk)
    print('frame', i)

