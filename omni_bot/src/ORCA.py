#!/usr/bin/env python
import rospy
import tf
from math import *
import sympy
from sympy import solve, Symbol
import numpy as np
from scipy.optimize import minimize
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates

bot_no = input("Enter bot number: ")
rospy.init_node('ORCA'+str(bot_no), anonymous=True)

name = 'bot' + str(bot_no)
goal_x = input("Enter goal x: ")
goal_y = input("Enter goal y: ")

pub1 = rospy.Publisher("/"+name+"/wheel_joint1_controller/command", Float64, queue_size=1)
pub2 = rospy.Publisher("/"+name+"/wheel_joint2_controller/command", Float64, queue_size=1)
pub3 = rospy.Publisher("/"+name+"/wheel_joint3_controller/command", Float64, queue_size=1)

goal_xprev = 0.0
goal_yprev = 0.0
sum_x = 0.0
sum_y = 0.0 
pos = 0
obstacle = []
tau = 1.8
v_max = 1.0
flag = 0
res = [0.0, 0.0]
radius = 0.42
sum_velx = 0
sum_vely = 0

########## Assuming we know the position and orientation of the robot, here we directly take the data from gazebo model_state ###########

def callback(data):
	global pos, bot_no, obstacle, v_max, tau, radius, name, sum_velx, sum_vely
	obstacle = []

	if pos == 0:
		for i in range (1, len(data.name)):
			if data.name[i] == name:
				pos = i

	## Getting bot data
	x = data.pose[pos].position.x
	y = data.pose[pos].position.y

	a = data.pose[pos].orientation.x
	b = data.pose[pos].orientation.y
	c = data.pose[pos].orientation.z
	d = data.pose[pos].orientation.w
	l = [a,b,c,d]

	vel_curx = data.twist[pos].linear.x
	vel_cury = data.twist[pos].linear.y

	roll, pitch, yaw = tf.transformations.euler_from_quaternion(l)

	for i in range (1, len(data.name)):
		if i!= pos:
			if dist(x,y,data.pose[i].position.x,data.pose[i].position.y)-2*radius < 2*v_max*tau:
				obstacle.append([data.pose[i].position.x, data.pose[i].position.y, data.twist[i].linear.x, data.twist[i].linear.y])

	w = 0

	v_1, v_2 = goal_pid(x,y,vel_curx,vel_cury)

	v_x, v_y = ORCA(v_1, v_2, x, y, vel_curx, vel_cury)

	v = np.array([[v_x],
				[v_y]])

	## Global frame to body fixed frame transform
	R_glob_to_loc = np.array([[cos(yaw), sin(yaw)],
							[-sin(yaw), cos(yaw)]])

	## Body frame to wheel velocity transform
	R_loc_to_wheel = np.array([[0, 0.57735026919, -0.57735026919],
							  [-2/3, 1/3, 1/3],
							  [1/(3*0.32), 1/(3*0.32), 1/(3*0.32)]])

	v_bar = np.matmul(R_glob_to_loc, v)

	v_bar = np.array([[v_bar[0]],
					 [v_bar[1]],
					 [w]])

	v = np.matmul(np.linalg.inv(R_loc_to_wheel), v_bar)

	# ###########			v_x, v_y, w are body linear and angular velocities. For now take them from user ##################
	# ###########			v1, v2, v3 are velocities at the COM of wheel. Divide by r_wheel to get ang vel for wheel ########

	v1 = v[0]/(0.065625)
	v2 = v[1]/(0.065625)
	v3 = v[2]/(0.065625)

	v1 = v1[0][0]
	v2 = v2[0][0]
	v3 = v3[0][0]

	pub1.publish(v1)
	pub2.publish(v2)
	pub3.publish(v3)

def ORCA(v_xpref, v_ypref, x, y, v_curx, v_cury):
	global v_max, tau, obstacle, res, flag, radius
	flag = 0
	v_opt = [v_xpref, v_ypref]
	u = []
	n = []
	pos = []
	cons = []
	con = 0
	r = 2*radius/tau

	if obstacle == []:
		return v_opt
	else:
		for i in range(len(obstacle)):
			## Center of the boundary circle in the velocity plane
			cx = (obstacle[i][0]-x)/tau
			cy = (obstacle[i][1]-y)/tau
			m = [0.0, 0.0]

			## Current Velocities
			v_cur = [v_curx, v_cury]

			## relative velocities
			v_relx = v_curx-obstacle[i][2]
			v_rely = v_cury-obstacle[i][3]

			## Finding the two tangents to the circle in vel plane
			m[0] = (cx*cy + np.sqrt((cx*cy)**2-(cx**2-r**2)*(cy**2-r**2)))/(cx**2-r**2)
			m[1] = (cx*cy - np.sqrt((cx*cy)**2-(cx**2-r**2)*(cy**2-r**2)))/(cx**2-r**2)

			## Points where tangents meet the circle (x1,y1), (x2,y2)
			x1 = (cy+cx/m[0])/(m[0]+1/m[0])
			y1 = m[0]*x1
			x2 = (cy+cx/m[1])/(m[1]+1/m[1])
			y2 = m[1]*x2

			## If the velocity lies between the two tangents (same side as center) 
			if (cy-m[0]*cx)*(v_rely-m[0]*v_relx)>0 and (cy-m[1]*cx)*(v_rely-m[1]*v_relx)>0:
				if area(v_relx, v_rely, cx, cy, x1, y1) + area(v_relx, v_rely, 0, 0, cx, cy) + area(v_relx, v_relx, 0, 0, x1, y1) == area(0, 0, cx, cy, x1, y1) and dist(cx,cy,v_relx,v_rely)>r:
					pass
				## If velocity not outside the circle between tangents
				else:
					## Finding the point closest to the rel vel on the boundary of the tangent/circle velocit obst region
					fun1 = lambda a1: (a1[0]-v_relx)**2+(a1[1]-v_rely)**2
					cons1 = [{'type': 'ineq', 'fun': lambda a1: np.sqrt(a1[0]**2 + a1[1]**2) - (np.sqrt(cx**2 + cy**2) - r)},
							 {'type': 'eq', 'fun': lambda a1: a1[1]-m[0]*a1[0]}]

					r1 = minimize(fun1, [x1,y1], constraints=cons1)
					px1 = r1.x[0]
					py1 = r1.x[1]

					fun2 = lambda a2: np.sqrt((a2[0]-v_relx)**2+(a2[1]-v_rely)**2)
					cons2 = [{'type': 'ineq', 'fun': lambda a2:  np.sqrt(a2[0]**2 + a2[1]**2) - (np.sqrt(cx**2 + cy**2) - r)},
							 {'type': 'eq', 'fun': lambda a2: a2[1]-m[1]*a2[0]}]

					r2 = minimize(fun2, [x2,y2], constraints=cons2)
					px2 = r2.x[0]
					py2 = r2.x[1]

					fun3 = lambda a3: np.sqrt((a3[0]-v_relx)**2+(a3[1]-v_rely)**2)
					cons3 = [{'type': 'eq', 'fun': lambda a3: area(a3[0], a3[1], cx, cy, x1, y1) + area(a3[0], a3[1], 0, 0, cx, cy) + 		  area(a3[0], a3[1], 0, 0, x1, y1) - area(0, 0, cx, cy, x1, y1)},
							 {'type': 'eq', 'fun': lambda a3: (a3[0]-cx)**2 + (a3[1]-cy)**2 - r**2}]

					r3 = minimize(fun3, [x1,y1], constraints=cons3)
					px3 = r3.x[0]
					py3 = r3.x[1]

					fun4 = lambda a4: np.sqrt((a4[0]-v_relx)**2+(a4[1]-v_rely)**2)
					cons4 = [{'type': 'eq', 'fun': lambda a4: area(a4[0], a4[1], cx, cy, x2, y2) + area(a4[0], a4[1], 0, 0, cx, cy) + 		  area(a4[0], a4[1], 0, 0, x2, y2) - area(0, 0, cx, cy, x2, y2)},
							 {'type': 'eq', 'fun': lambda a4: (a4[0]-cx)**2 + (a4[1]-cy)**2 - r**2}]

					r4 = minimize(fun4, [x2,y2], constraints=cons4)
					px4 = r4.x[0]
					py4 = r4.x[1]

					if dist(v_relx,v_rely,px1,py1)<=dist(v_relx,v_rely,px2,py2) and dist(v_relx,v_rely,px1,py1)<=dist(v_relx,v_rely,px3,py3) and dist(v_relx,v_rely,px1,py1)<=dist(v_relx,v_rely,px4,py4):
						d_min = dist(v_relx,v_rely,px1,py1)
						u.append([px1-v_relx, py1-v_rely])
						n.append([(px1-v_relx)/d_min, (py1-v_rely)/d_min])

					elif dist(v_relx,v_rely,px2,py2)<dist(v_relx,v_rely,px1,py1) and dist(v_relx,v_rely,px2,py2)<=dist(v_relx,v_rely,px3,py3) and dist(v_relx,v_rely,px2,py2)<=dist(v_relx,v_rely,px4,py4):
						d_min = dist(v_relx,v_rely,px2,py2)
						u.append([px2-v_relx, py2-v_rely])
						n.append([(px2-v_relx)/d_min, (py2-v_rely)/d_min])

					elif dist(v_relx,v_rely,px3,py3)<dist(v_relx,v_rely,px2,py2) and dist(v_relx,v_rely,px3,py3)<dist(v_relx,v_rely,px1,py1) and dist(v_relx,v_rely,px3,py3)<=dist(v_relx,v_rely,px4,py4):
						d_min = dist(v_relx,v_rely,px3,py3)
						u.append([px3-v_relx, py3-v_rely])
						n.append([(px3-v_relx)/d_min, (py3-v_rely)/d_min])

					elif dist(v_relx,v_rely,px4,py4)<dist(v_relx,v_rely,px1,py1) and dist(v_relx,v_rely,px4,py4)<dist(v_relx,v_rely,px2,py2) and dist(v_relx,v_rely,px4,py4)<=dist(v_relx,v_rely,px3,py3):
						d_min = dist(v_relx,v_rely,px4,py4)
						u.append([px4-v_relx, py4-v_rely])
						n.append([(px4-v_relx)/d_min, (py4-v_rely)/d_min])
			
		if u == []:
			return v_opt[0], v_opt[1]

		else:
			v_opt = np.array(v_opt, dtype=Float64)
			u = np.array(u, dtype=Float64)
			n = np.array(n, dtype=Float64)

			## Minimizing the distance to the optimal velocity value from PID 
			fun = lambda v: (v[0]-v_opt[0])**2 + (v[1]-v_opt[1])**2

			cons = [{'type': 'ineq', 'fun': lambda v: v_max**2 - v[0]**2 - v[1]**2},]

			for i in range(len(u)):
				if i == len(u)-1:
					cons = cons + [{'type': 'ineq', 'fun': lambda v: np.dot((v-(v_cur+0.5*u[i])),n[i])}]
				else:
					cons = cons + [{'type': 'ineq', 'fun': lambda v: np.dot((v-(v_cur+0.5*u[i])),n[i])},]

			r = minimize(fun, v_opt, constraints=cons)
			return r.x[0], r.x[1]

## Distance between two points
def dist(x1, y1, x2, y2):
	d = np.sqrt((x2-x1)**2 + (y2-y1)**2)
	return d

## Distance of point from line
def dist_from_line(x0, y0, m, c):
	d = abs((y0-m*x0-c)/sqrt(1+m**2))
	return d

## Area of triangle
def area(x1, y1, x2, y2, x3, y3):
	a = (x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))/2
	return a

## Optimal velocities given by the PID
def goal_pid(x,y,vel_curx,vel_cury):
	global goal_x, goal_y, goal_xprev, goal_yprev, sum_x, sum_y, v_max

	kp = 0.5
	ki = 0.000085
	kd = 0.01

	vx = kp*(goal_x-x) + kd*(goal_x-x-goal_xprev) + ki*sum_x
	vy = kp*(goal_y-y) + kd*(goal_y-y-goal_yprev) + ki*sum_y

	if vx>1:
		vx = 1
	if vx<-1:
		vx = -1
	if vy>1:
		vy = 1
	if vy<-1:
		vy = -1

	goal_xprev = goal_x-x
	goal_yprev = goal_y-y
	sum_x += goal_x-x
	sum_y += goal_y-y

	if abs(goal_x-x)<=0.1:
		sum_x = 0
	if abs(goal_y-y)<=0.1:
		sum_y = 0
	
	return vx,vy

def listener ():
	rospy.Subscriber("/gazebo/model_states", ModelStates, callback, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass