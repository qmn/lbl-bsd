#!/usr/bin/python

# where.py
#
# Quan Nguyen
# 14 April 2012
#
# Plots the GPS location, based on information received over serial.

import sys
import re
import numpy
import matplotlib.pyplot as plt

d = "/dev/ttyUSB0"
c = 0
n = 0
fmt = "{:2.4f}"

if (len(sys.argv) > 1):
	d = sys.argv[0]

f = open(d, "r")

plt.ion()

w = 50

from collections import deque

lat = deque(0*[0.], w)
lon = deque(0*[0.], w)

line, = plt.plot(lat, lon, linewidth=2.0, color="r")
line.axes.set_xlim(15400, 15500)
line.axes.set_ylim(52400, 52600)

while True:
	l = f.readline()
	if ((len(l) - 2) % 4 == 0):
		a = []
		b = []
		for i in xrange(0, len(l) / 4):
			j = i * 4
			k = j + 4
			v = int(l[j:k], 16)
			b.append(v)
			a.append(fmt.format(v/8192.))
		lat.append(b[3])
		lon.append(b[4])
		print "\t".join(a)
	line.set_xdata(lon)
	line.set_ydata(lat)
	plt.draw()

