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
	
	# quad function for fitting
	# フィッティング用二次関数
	def fit_quad(self, x, a, b, c):
		return a*x**2 + b*x + c

	# 
	def get_cutoff_val_max(self, index, len_scan):
		max_left  = 0.5 #[m]
		max_front = 1.0 #[m]
		max_right = 0.5 #[m]

		x = [0.0, 0.5, 1.0]
		y = [max_left, max_front, max_right]

		res = opt.curve_fit(self.fit_quad, x, y)
		a = res[0][0]
		b = res[0][1]
		c = res[0][2]

		# センサ値の番号を定義域内(0~1)にマッピング
		index_map = self.map(index, 0.0, len_scan, 0.0, 1.0)

		return a*index_map**2 + b*index_map + c

	def get_cutoff_val_min(self, index, len_scan):
		min_left  = 0.3 #[m]
		min_front = 0.5 #[m]
		min_right = 0.3 #[m]

		x = [0.0, 0.5, 1.0]
		y = [min_left, min_front, min_right]

		res = opt.curve_fit(self.fit_quad, x, y)
		a = res[0][0]
		b = res[0][1]
		c = res[0][2]

		index_map = self.map(index, 0.0, len_scan, 0.0, 1.0)

		return a*index_map**2 + b*index_map + c

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

	# def sigmoid(self, a):
	# 	return (1/(1+math.e**((a/0.01-30)*-1)))+2

	# callback function
	# > 
	def subLidarScanCallback(self, scan):
		# LIDAR's spec
		# range_min = 0.1
		# range_max = 3.5

		# nomalize scan value
		# 
		index = [i for i in range(len(scan.data))]
		scan_norm = []
		for i in range(len(scan.data)):
			range_max   = self.get_cutoff_val_max(index[i], len(index))
			range_min   = self.get_cutoff_val_min(index[i], len(index))
			scan_cutted = self.cut( scan.data[i], range_min, range_max )
			scan_norm.append( self.normalize(scan_cutted, range_min, 1.5) )
			# rospy.loginfo("val[%d]:%f", i, scan_norm[i])

		# difine controll value
		linVel = 1 * sum(scan_norm)/len(scan_norm)
		angVel = 1 - 2 * linVel
		rospy.loginfo("linVel:%f angVel:%f", linVel, angVel)

		# define direction of angular movement
		leftVal  = 0
		rightVal = 0
		for i in range(len(scan.data)):
			if i < len(scan.data)/2:
				leftVal  = leftVal  + scan_norm[i]
			else:
				rightVal = rightVal + scan_norm[i]
		
		ang_dir = 1 if leftVal > rightVal else -1

		self.pub_lin_vel.publish(linVel)
		self.pub_ang_vel.publish(angVel*ang_dir)

if __name__ ==  '__main__':
	try:
		prcpl = pubRobotControlParam_LIDAR()
		rospy.spin()
	except rospy.ROSInterruptException: pass
