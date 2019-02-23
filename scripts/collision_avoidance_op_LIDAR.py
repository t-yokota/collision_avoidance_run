#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

import math
from operator import mul
from scipy import optimize as opt

class pubRobotControlParam_LIDAR:
	# 初期化関数
	def __init__(self):
		rospy.init_node('collision_avoidance_op', anonymous=True)
		rospy.Subscriber('/scan/downsampled/forward/ranges', Float32MultiArray, self.subLidarScanCallback)
		self.pub_lin_vel = rospy.Publisher('/vel/lin', Float32, queue_size=10)
		self.pub_ang_vel = rospy.Publisher('/vel/ang', Float32, queue_size=10)
	
	# cutoff functioin
	# > 設定範囲内にセンサ値を丸める
	def cut(self, x, f_min, f_max):
		if   float(f_min) > x:
			return f_min
		elif float(f_max) < x or x == float('inf'):
			return f_max
		else:
			return x

	# nomalize function
	# > 最大1，最小0で正規化
	def normalize(self, x, range_min, range_max):
		return (x - float(range_min))/(float(range_max) - float(range_min))

	# mapping function
	# > 範囲a内の値を範囲b内に取り直す
	def map(self, x, in_min, in_max, out_min, out_max):
		return (x - float(in_min)) * (float(out_max) - float(out_min))/(float(in_max) - float(in_min)) + float(out_min)

	# callback function
	# > LIDARの値を用いた障害物回避走行
	def subLidarScanCallback(self, scan):
		# define the using range of lidar value (0.5~1.0m)
		range_cutoff_max = 1.0
		range_cutoff_min = 0.5

		# nomalize scan value
		scan_norm = []
		for i in range(len(scan.data)):
			scan_norm.append(self.normalize(self.cut(scan.data[i], range_cutoff_min, range_cutoff_max), range_cutoff_min, range_cutoff_max))
			# rospy.loginfo("val[%d]:%f", i, scan_norm[i])
		
		# difine controll value
		linVel = sum(scan_norm)/len(scan_norm)
		angVel = 1.0 - linVel

		# define direction of angular movement
		l = 0
		r = 0
		for i in range(len(scan_norm)):
			if i < len(scan_norm)/2:
				l = l + (1-scan_norm[i])**2
			else:
				r = r + (1-scan_norm[i])**2
		angDir = 1 if l < r else -1
		
		rospy.loginfo("linVel:%f angVel:%f, angDir:%f", linVel, angVel, angDir)

		self.pub_lin_vel.publish(linVel*0.7)
		self.pub_ang_vel.publish(angVel*angDir*0.9)

if __name__ ==  '__main__':
	try:
		prcpl = pubRobotControlParam_LIDAR()
		rospy.spin()
	except rospy.ROSInterruptException: pass
