#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

import math
from operator import mul

class pubRobotControlParam_LIDAR:
	# 初期化関数
	def __init__(self):
		rospy.init_node('collision_avoidance_op', anonymous=True)
		rospy.Subscriber('/scan/downsampled/forward/ranges', Float32MultiArray, self.subLidarScanCallback)
		self.pub_lin_vel = rospy.Publisher('/vel/lin', Float32, queue_size=10)
		self.pub_ang_vel = rospy.Publisher('/vel/ang', Float32, queue_size=10)
	
	# cutoff functioin
	# > 設定範囲内にセンサ値を丸める
	def cut(self, x, f):
		return f if float(f) < x or x == float('inf') else x

	# nomalize function
	# > 最大１，最小０で正規化
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

		return math.exp(-1 * (x_map - u)**2 / (2 * s))/math.sqrt(2 * math.pi * s)

	# callback function
	# > 
	def subLidarScanCallback(self, scan):
		# LIDAR's spec
		range_min = 0.1
		range_max = 3.5

		# nomalize scan value
		range_cutoff = 0.7
		scan_norm = []
		for i in range(len(scan.data)):
			scan_norm.append(1 - self.normalize( self.cut(scan.data[i], range_cutoff), range_min, range_cutoff ))
			# rospy.loginfo("val[%d]:%f", i, scan_norm[i])

		# get weight function
		x = [i for i in range(len(scan_norm))]
		w = []
		for i in range(len(x)):
			w.append(self.gaussian_weight(x[i], len(x)))
			# rospy.loginfo("val[%d]:%f", i, w[i])
		
		# difine controll value
		angVel = 4 * sum(map(mul, scan_norm, w))/len(scan_norm)
		linVel = 0.8 - 2 * angVel
		rospy.loginfo("linVel:%f angVel:%f", linVel, angVel)

		# define direction of angular movement
		leftVal  = 0
		rightVal = 0
		for i in range(len(scan_norm)):
			if i < len(scan_norm)/2:
				leftVal  = leftVal  + scan_norm[i] * w[i]
			else:
				rightVal = rightVal + scan_norm[i] * w[i]
		
		ang_dir = 1 if leftVal < rightVal else -1

		self.pub_lin_vel.publish(linVel)
		self.pub_ang_vel.publish(angVel*ang_dir)

if __name__ ==  '__main__':
	try:
		prcpl = pubRobotControlParam_LIDAR()
		rospy.spin()
	except rospy.ROSInterruptException: pass
