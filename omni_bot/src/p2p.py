#!/usr/bin/env python
import rospy
import tf
from math import *
from sympy import solve
from sympy import Symbol
import time
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates

rospy.init_node('p2p', anonymous=True)

bot_no = input("Enter bot number: ")
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
i = 0


########## 		Assuming we know the position and orientation of the robot, here we directly take the data from gazebo 		model_state 			##########################################

def callback(data):
	global i, bot_no
	w = 0

	v1 = Symbol('v1')
	v2 = Symbol('v2')
	v3 = Symbol('v3')

	if i ==0:
		for i in range (len(data.name)):
			if data.name[i] == name:
				break
	x = data.pose[i].position.x
	y = data.pose[i].position.y
	print(x,y)

	a = data.pose[i].orientation.x
	b = data.pose[i].orientation.y
	c = data.pose[i].orientation.z
	d = data.pose[i].orientation.w
	l = [a,b,c,d]

	roll, pitch, yaw = tf.transformations.euler_from_quaternion(l)

	v_x, v_y = goal_pid(x,y)

	###########			v_x, v_y, w are body linear and angular velocities. For now take them from user ##################
	###########			v1, v2, v3 are velocities at the COM of wheel. Divide by r_wheel to get ang vel for wheel ########

	sol = solve([-v_x+v1*sin(yaw)+v2*sin(np.pi/3-yaw)-v3*sin(np.pi/3+yaw), -v_y-v1*cos(yaw)+v2*cos(np.pi/3-yaw)+v3*cos(np.pi/3+yaw), -w+ (v1+v2+v3)/0.35], dict=True)

	v1 = sol[0][v1]/0.065625
	v2 = sol[0][v2]/0.065625
	v3 = sol[0][v3]/0.065625

	pub1.publish(v1)
	pub2.publish(v2)
	pub3.publish(v3)


def goal_pid(x,y):
	global goal_x, goal_y, goal_xprev, goal_yprev, sum_x, sum_y

	kp = 1
	ki = 0.001
	kd = 0.5

	vx = kp*(goal_x-x) + kd*(goal_x-goal_xprev) + ki*sum_x
	vy = kp*(goal_y-y) + kd*(goal_y-goal_yprev) + ki*sum_y

	goal_xprev = goal_x
	goal_yprev = goal_y
	sum_x += goal_x-x
	sum_y = goal_y-y

	if vx>2:
		vx = 2
	if vx<-2:
		vx = -2
	if vy>2:
		vy = 2
	if vy<-2:
		vy = -2

	return vx,vy


def listener ():
	rospy.Subscriber("/gazebo/model_states", ModelStates, callback, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass