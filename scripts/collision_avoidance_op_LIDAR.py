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

	# gaussian function for weighting
	# > (len_scan)個のLIDAR値の重み付けに用いるガウス関数
	def gaussian_weight(self, x, len_scan):
		# 分散：分布の形（sが大きいほど扁平）
		s = 0.1
		# 平均：uを与えるとx方向に移動→関数を第１象限に収めておく
		u = 3.0 * math.sqrt(s)
		# 定義域
		x_min = 0.0
		x_max = 2 * u
		# センサ値の番号を定義域内にマッピング
		x_map = self.map(x, 0.0, len_scan, x_min, x_max)

		return math.exp(-1 * (x_map - u)**2 / (2 * s))

	# quadratic function for weighting
	# > (len_scan)個のLIDAR値の重み付けに用いる2次関数
	def quad_func(self, x, a, b, c):
			return a*x**2 + b*x + c
	def quadratic_weight(self, index, len_scan):
		w_left  = 0.3
		w_front = 1.0
		w_right = 0.3

		x = [0.0, 0.5, 1.0]
		y = [w_left, w_front, w_right]

		res = opt.curve_fit(self.quad_func, x, y)
		a = res[0][0]
		b = res[0][1]
		c = res[0][2]

		# センサ値の番号を定義域内にマッピング
		index_map = self.map(index, 0.0, len_scan, 0.0, 1.0)
		
		w = self.quad_func(index_map, a, b, c)
		return w

	# quartic function for weighting
	# > (len_scan)個のLIDAR値の重み付けに用いる4次関数
	def quart_func(self, x, a, b, c, d, e):
			return a*x**4 + b*x**3 + c*x**2 + d*x + e
	def quartic_weight(self, index, len_scan):
		w_left  = 0.3
		w_front = 1.0
		w_right = 0.3

		x = [0.0, 0.4, 0.5, 0.6, 1.0]
		y = [w_left, w_front, w_front, w_front, w_right]

		res = opt.curve_fit(self.quart_func, x, y)
		a = res[0][0]
		b = res[0][1]
		c = res[0][2]
		d = res[0][3]
		e = res[0][4]

		# センサ値の番号を定義域内にマッピング
		index_map = self.map(index, 0.0, len_scan, 0.0, 1.0)
		
		w = self.quart_func(index_map, a, b, c, d, e)
		return w

	# callback function
	# > 
	def subLidarScanCallback(self, scan):
		# LIDAR's spec
		# range_min = 0.1
		# range_max = 3.5

		# nomalize scan value
		range_cutoff_max = 1.0
		range_cutoff_min = 0.5

		scan_norm = []
		for i in range(len(scan.data)):
			scan_norm.append(self.normalize( self.cut(scan.data[i], range_cutoff_min, range_cutoff_max), range_cutoff_min, range_cutoff_max))
			# rospy.loginfo("val[%d]:%f", i, scan_norm[i])

		# get weight function
		index = [i for i in range(len(scan_norm))]
		w = []
		for i in range(len(index)):
			w.append(self.quadratic_weight(index[i], len(index)))
			# w.append(self.quartic_weight(index[i], len(index)))
			# w.append(self.gaussian_weight(index[i], len(index)))
			# rospy.loginfo("val[%d]:%f", i, w[i])
		
		# difine controll value
		linVel = sum(map(mul, scan_norm, w))/len(scan_norm)
		angVel = 0.7 - linVel

		if min(scan_norm[60:120]) == 0:
			linVel = 0.0
			angVel = 2.0

		rospy.loginfo("linVel:%f angVel:%f", linVel, angVel)

		# define direction of angular movement
		l = 0
		r = 0
		for i in range(len(scan_norm)):
			if i < len(scan_norm)/2:
				l = l + (1-scan_norm[i])**10
			else:
				r = r + (1-scan_norm[i])**10
		ang_dir = 1 if l < r else -1
		
		self.pub_lin_vel.publish(linVel * 0.7)
		self.pub_ang_vel.publish(angVel*ang_dir * 0.5)

if __name__ ==  '__main__':
	try:
		prcpl = pubRobotControlParam_LIDAR()
		rospy.spin()
	except rospy.ROSInterruptException: pass
