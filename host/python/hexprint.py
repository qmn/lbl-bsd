#!/usr/bin/python

# hexprint.py
#
# Quan Nguyen
# 14 April 2012
#
# Parses a hex string into an arbitrary number of 3.13 format fixed-point
# integers, then prints to screen. Prints from /dev/ttyUSB0 by default,
# but supports a file specified from the command line too.

import sys
import re

d = "/dev/ttyUSB0"
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
		print "\t".join(a)

