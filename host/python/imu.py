#!/usr/bin/python
 
# imu.py
#
# Quan Nguyen
# 14 April 2012
#
# An early IMU -> DCM script.
#
# interprets the accelerometer, magnetometer, and gyroscope string.
# it's formatted like
# A27X0310YDDE0Z3560-M00XFEB7Y000EZFF57-G0FXFFC3Y00DFZ0058
# [  accelerometer ] [  magnetometer  ] [ rate gyroscope ]
#
# and displays a teapot visual, based on the DCM calculations from these
# readings.

import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

def aread(a):
	b = int(a, 16)
	if (b > 2 ** 11):
		b = b - 2 ** 12
	return b

def mread(a):
	b = int(a, 16)
	if (b > 2 ** 15):
		b = b - 2 ** 16
	return b

def gread(a):
	b = int(a, 16)
	if (b > 2 ** 15):
		b = b - 2 ** 16
	return b

alpha = 0.9
def lpf(a, b):
	return int(alpha * a + (1-alpha) * b)

# Accelerometer calibration
bias_x = 11.12
bias_y =  0.72
bias_z =  4.12

sens_x = 1022.40
sens_y = 1029.44
sens_z = 1002.12

ax = 0
ay = 0
az = 0

mx = 0
my = 0
mz = 0

gx = 0
gy = 0
gz = 0

log_file = open('/dev/ttyUSB0', 'r');

def print_debug():
	global ax, ay, az, mx, my, mz, gx, gy, gz
	print "A = ({:5.3f}, {:5.3f}, {:5.3f})".format(ax, ay, az)
	print "M = ({:5.3f}, {:5.3f}, {:5.3f})".format(mx, my, mz)
	print "G = ({:5.3f}, {:5.3f}, {:5.3f})".format(gx, gy, gz)

def sample(log):
	global ax, ay, az, mx, my, mz, gx, gy, gz
	global bias_x, bias_y, bias_z, sens_x, sens_y, sens_z

	line = log.readline()[1:]
	while (len(line) < 54):
		line = log.readline()[1:]
	ax1 = ax
	ay1 = ay
	az1 = az

	ax = aread(line[4:7])
	ay = aread(line[9:12])
	az = aread(line[14:17])

	ax = lpf(ax, ax1)
	ay = lpf(ay, ay1)
	az = lpf(az, az1)

	ax = (ax - bias_x) / sens_x
	ay = (ay - bias_y) / sens_y
	az = (az - bias_z) / sens_z


	mx1 = mx
	my1 = my
	mz1 = mz

	mx = mread(line[23:27])
	my = mread(line[28:32])
	mz = mread(line[33:37])

	mx = lpf(mx, mx1)
	my = lpf(my, my1)
	mz = lpf(mz, mz1)

	mt = pow(mx ** 2 + my ** 2 + mz ** 2, 0.5)
	mx = mx/mt
	my = my/mt
	mz = mz/mt

	gx1 = gx
	gy1 = gy
	gz1 = gz
	
	gx = gread(line[42:46])
	gy = gread(line[47:51])
	gz = gread(line[52:56])

#	gx = lpf(gx, gx1)
#	gy = lpf(gy, gy1)
#	gz = lpf(gz, gz1)


pitch = yaw = roll = 0
import math
from math import *
import numpy
from numpy import linalg

rot_matrix = numpy.matrix('0.0 0.0 0.0; 0.0 0.0 0.0; 0.0 0.0 0.0')

def update():
	global pitch, yaw, roll
	global ax, ay, az, mx, my, mz, gx, gy, gz
	# berkeley_magnetic_declination = (-14 + 6/60 + 20/3600) # 14deg, 6', 20"
	# mm = pow(mx ** 2 + my ** 2, 0.5)
	# yaw = (math.degrees(math.atan2(my/mm, mx/mm) + math.pi) + berkeley_magnetic_declination) % 360
	# print str(yaw) + " degrees"

	# Pitch and roll from the EECS C149 Lab 1
	roll = math.atan2(ay, az)
	pitch = math.atan(ax / (ay * math.sin(roll) + az * math.cos(roll))) 
	roll = math.degrees(roll)
	pitch = math.degrees(pitch)

	print "roll = {r}\t pitch = {p}".format(r=roll, p=pitch)

def calibrate_rotation_matrix():
	global ax, ay, az
	global rot_matrix
	# theta: pitch
	# phi: roll
	# psi: yaw
	roll = math.atan2(ay, az)
	pitch = math.atan(ax / (ay * math.sin(roll) + az * math.cos(roll))) 
	yaw = 0.0
	rot_matrix = numpy.matrix([[cos(pitch)*cos(yaw), sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw), cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw)],
	                          [cos(pitch)*sin(yaw), sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw), cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw)],
														[-sin(pitch)        , sin(roll)*cos(pitch)                            , cos(roll)*cos(pitch)                             ]])
	# done

def dcm():
	global gx, gy, gz
	global rot_matrix
	gyro_output_rate = 100 # hertz
	gyro_output_period = 1.0 / gyro_output_rate # seconds

	# remember that this is a small angle approximation.
	# gx, gy, gz must all be rather small

	accel_vect = numpy.matrix([[0], [0], [1]])
	accel_real = numpy.matrix([[ax], [ay], [az]])

	accel_err = (rot_matrix * accel_vect) - accel_real

	print "ex={x}\tey={y}\tez={z}".format(x=accel_err[0,0], y=accel_err[1,0], z=accel_err[2,0])

	dgx = gx * gyro_output_period * 0.00875
	dgy = gy * gyro_output_period * 0.00875
	dgz = gz * gyro_output_period * 0.00875

	print "gx={x}\tgy={y}\tgz={z}".format(x=dgx, y=dgy, z=dgz)

	# generate the infinitesimal rotation matrix, and hit the rotation matrix
	infinitesimal_rot_matrix = numpy.matrix([[1       , -1 * dgz, dgy     ], 
	                                         [dgz     , 1       , -1 * dgx],
																					 [-1 * dgy, dgx     , 1       ]])
	rot_matrix = rot_matrix * infinitesimal_rot_matrix

def renorm():
	global rot_matrix
	x = rot_matrix[0] # extract the X row
	y = rot_matrix[1] # and the y row
	err = numpy.vdot(x, y) # the value is in a matrix    - compute the error dot product
	xo = x - (err/2)*y # remove half the error from each vector
	yo = y - (err/2)*x # remove the other half of the error from this vector
	zo = numpy.cross(xo, yo) # create a new vector for Z, by taking the cross product
	xo = xo / sqrt(numpy.vdot(xo, xo)) # normalize these vectors to have length 1
	yo = yo / sqrt(numpy.vdot(yo, yo))
	zo = zo / sqrt(numpy.vdot(zo, zo))
	rot_matrix[0] = xo # restore the rotation matrix
	rot_matrix[1] = yo
	rot_matrix[2] = zo

# here is my OpenGL codeee

# This function executes every 10 ms
def timed_func(derp):
	global yaw, pitch, roll
	glLoadIdentity()
	glRotatef(yaw, 0.0, 1.0, 0.0)
	glRotatef(pitch, 0.0, 0.0, 1.0)
	glRotatef(roll, -1.0, 0.0, 0.0)
	glutPostRedisplay() # Makes it redisplay
	glutTimerFunc(10, timed_func, 0) # Call again in 10ms

def display_func():
	glClear(GL_COLOR_BUFFER_BIT) # Clear the screen
	glutWireTeapot(0.5)          # Draw the teapot!
	sample(log_file)     			   # Get some data
	update()
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

up = numpy.matrix([[0], [0], [1]])
calib_count = 100
until_recalib = calib_count
sample(log_file)
calibrate_rotation_matrix()
while True:
	sample(log_file)

	if until_recalib == 0:
		print "RECALIB"
		calibrate_rotation_matrix()
		until_recalib = calib_count
	dcm()
	renorm()
	rup = linalg.solve(rot_matrix, up)
	rup = rup / math.sqrt(numpy.vdot(rup, rup))
	print rup

	roll = math.atan2(rup[1], rup[2])
	pitch = math.atan(rup[0] / (rup[1] * math.sin(roll) + rup[2] * math.cos(roll))) 
	roll = math.degrees(roll)
	pitch = math.degrees(pitch)

	print "roll = {r}\t pitch = {p}".format(r=roll, p=pitch)


	print rot_matrix
	until_recalib -= 1

