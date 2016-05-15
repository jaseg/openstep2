#!/usr/bin/env python3

import serial
import math
import time
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import threading

w = 400

ser = serial.Serial('/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0', 500000)

def rxgen(ser, n):
    while True:
        line = ser.readline()[:-1]
        if len(line) == 3*n:
            yield [int(line[i:i+3], 16)-2048 for i in range(0, 32*3, 3)]

time.sleep(0.5)
ser.flush()
ser.readline()
ser.readline()
n = len(ser.readline()[:-1])//3
ydata = np.zeros((n, w))

plx = math.ceil(math.sqrt(n))
ply = math.ceil(n/plx)

app = QtGui.QApplication([])
view = pg.GraphicsView()
gl = pg.GraphicsLayout()
view.setCentralWidget(gl)
view.show()
view.resize(800,800)

ps = [ gl.addPlot(row=x, col=y) for x in range(plx) for y in range(ply) if plx*y+x < n ]
pls = [ p.plot(np.arange(w), np.zeros(w)) for p in ps ]
for p in ps:
	p.setYRange(-2048, 2047)
print(len(ps), len(pls))

def populate():
    global n, ser, ydata, pls
    for i, frame in enumerate(rxgen(ser, n)):
        ydata[:, :-1] = ydata[:, 1:]
        ydata[:, -1] = np.array(frame)
        for j, p in enumerate(pls):
            p.setData(np.arange(w), ydata[j, :])
        print('frame', i)
th = threading.Thread(target=populate, daemon=True)
th.start()

# Fix Ctrl+C behavior
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

