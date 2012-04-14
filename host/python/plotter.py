#!/usr/bin/python

# plotter.py
#
# Quan Nguyen
# 14 April 2012
#
# Plots the first two hex values it receives.
# When the first two values are yaw and pitch, it will plot them over time.

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


def parse313(s):
	if not re.search("^[0-9A-Fa-f]{4}$", s):
		return 1337
	n = int(s, 16)
	if (n >= 0x8000):
		n = n - 0x10000
	n = n / 8192.0
	return n

plt.ion()

w = 50

from collections import deque

x = deque(0*[0.], w)
h = deque(0*[0.], w)
g = deque(0*[0.], w)

line_a,line_b = plt.plot(x, h, x, g)

while True:
	n = n + 1
	l = f.readline()
	if ((len(l) - 2) % 4 == 0):
		a = []
		b = []
		for i in xrange(0, len(l) / 4):
			j = i * 4
			k = j + 4
			v = parse313(l[j:k])
			b.append(v)
			a.append(fmt.format(v))
		h.append(b[0])
		g.append(b[1])
		x.append(n)
		print "\t".join(a)
	line_a.set_ydata(h)
	line_b.set_ydata(g)
	line_a.set_xdata(x)
	line_b.set_xdata(x)
	line_a.axes.set_ylim(min(h) - 0.1, max(h) + 0.1)
	line_a.axes.set_xlim(n - w, n)
	line_b.axes.set_ylim(min(g) - 0.1, max(g) + 0.1)
	line_b.axes.set_xlim(n - w, n)
	plt.draw()

