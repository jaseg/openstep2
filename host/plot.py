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
        if len(line) == 3*32: #n:
            yield list(reversed([int(line[i:i+3], 16)-2048 for i in range(0, 32*3, 3)]))[s:e]
#            yield [int(line[i:i+3], 16)-2048 for i in range(0, 32*3, 3)]

time.sleep(0.5)
ser.flush()
ser.readline()
ser.readline()
#n = len(ser.readline()[:-1])//3
s,e = 0,32
n = e-s
ydata = np.zeros((n, w))

app = QtGui.QApplication([])

gl = pg.GraphicsLayout()
cgl = pg.GraphicsLayout()
win = pg.GraphicsView()
win.setCentralWidget(gl)
win.show()
win.resize(800,800)

single_selected = 0

class ChannelPlot(pg.PlotItem):
    def __init__(self, channel):
        super(ChannelPlot, self).__init__()
        self.channel = channel

    def mouseDoubleClickEvent(self, ev):
        global single_selected, cgl, win
        single_selected = self.channel
        gl.setVisible(False)
        cgl.setVisible(True)
        win.setCentralWidget(cgl)

class SinglePlot(pg.PlotItem):
    def keyPressEvent(self, ev):
        if ev.key() == QtCore.Qt.Key_Escape:
            gl.setVisible(True)
            cgl.setVisible(False)
            win.setCentralWidget(gl)

sp = SinglePlot()
spp = sp.plot(np.arange(w), np.zeros(w))
sp.setYRange(-2048, 2047)
cgl.addItem(sp)

cps = [ (ChannelPlot(x*8+y), x, y) for x in range(4) for y in range(8) ]
for p, x, y in cps:
    gl.addItem(p, row=y, col=x)
pls = [ p.plot(np.arange(w), np.zeros(w)) for p,_x,_y in cps ]
for p,_x,_y in cps:
	p.setYRange(-2048, 2047)

def populate():
    global n, ser, ydata, pls
    for i, frame in enumerate(rxgen(ser, n)):
        ydata = np.concatenate([ydata[:, 1:], np.array(frame).reshape(n, 1)], axis=1)
        if gl.isVisible():
            for j, p in enumerate(pls):
                p.setData(np.arange(w), ydata[j, :])
        elif cgl.isVisible():
            spp.setData(np.arange(w), ydata[single_selected, :])
#        print('frame', i)
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

