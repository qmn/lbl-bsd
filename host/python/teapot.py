#!/usr/bin/python

# teapot.py
#
# Quan Nguyen
# 14 April 2012
#
# Teapot visualization for Berkeley Solar Drone
# Requires PyOpenGL

import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

log_file = open('/dev/ttyUSB0', 'r');

def parse213(s):
	n = int(s, 16)
	if (n >= 0x8000):
		n = n - 0x10000
	n = n / 8192.0
	return n

pitch = yaw = roll = 0

def sample(f):
	global pitch, yaw, roll
	l = f.readline()
	if (len(l) < 16):
		return
	pitch = degrees(parse213(l[8:12]))
	yaw = degrees(parse213(l[4:8]))
	roll = degrees(parse213(l[12:16]))

from math import *

# here is my OpenGL codeee

# This function executes every 10 ms
def timed_func(derp):
	global yaw, pitch, roll
	yaw = 0
	glLoadIdentity()
	glRotatef(yaw, 0.0, 1.0, 0.0)
	glRotatef(pitch, 0.0, 0.0, -1.0)
	glRotatef(roll, -1.0, 0.0, 0.0)
	glutPostRedisplay() # Makes it redisplay
	glutTimerFunc(5, timed_func, 0) # Call again in 10ms

def display_func():
	glClear(GL_COLOR_BUFFER_BIT) # Clear the screen
	glutWireTeapot(0.5)          # Draw the teapot!
	sample(log_file)     			   # Get some data
	glutSwapBuffers()            # Switch buffers...?

def visualize():
	glutInit()
	glutInitWindowSize(500, 500)
	glutInitWindowPosition(100, 100)

	win = glutCreateWindow("Pololu IMU Visualization - Light Basin Laboratory")

	glutDisplayFunc(display_func)
	glClearColor(0.0, 0.0, 0.0, 0.0)
	glutTimerFunc(10, timed_func, 0)
	glutMainLoop()

sample(log_file)
visualize()

