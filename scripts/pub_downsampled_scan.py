#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2019, Tomohiro Yokota.
# 

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class defSamplingDegree:
	min = 0
	max = 0

deg1 = defSamplingDegree()
deg2 = defSamplingDegree()

class pubDownSampledScan:
	def __init__(self):
		rospy.init_node('pubDownSampledScan', anonymous=True)
		rospy.Subscriber('/scan', LaserScan, self.scanCallback)
		self.pub_scan_downsampled_forward   = rospy.Publisher('/scan/downsampled/forward', LaserScan, queue_size=10)
		self.pub_ranges_downsampled_forward = rospy.Publisher('/scan/downsampled/forward/ranges', Float32MultiArray, queue_size=10)

	def scanCallback(self, scan):
		# define the sampling range of degree
		deg1.min = 0
		deg1.max = 90
		deg2.min = 270
		deg2.max = 360
		# get data per (step) degree 
		step = 10

		# instance for RViz
		downsampled_scan   = LaserScan()
		# list for controlling robot
		downsampled_ranges = Float32MultiArray()

		# copy instance
		downsampled_scan.header          = scan.header
		downsampled_scan.angle_min       = scan.angle_min
		downsampled_scan.angle_max       = scan.angle_max	
		downsampled_scan.angle_increment = scan.angle_increment
		downsampled_scan.time_increment  = scan.time_increment
		downsampled_scan.scan_time       = scan.scan_time
		downsampled_scan.range_min       = scan.range_min
		downsampled_scan.range_max       = scan.range_max
		downsampled_scan.intensities     = scan.intensities
		
		a = list()
		b = list()

		# down sampling data
		# get scan data in deg1 (from 179 to 0)
		for i in reversed(range(0, 180)):
			if deg1.min <= i and i <= deg1.max-1:
				if i % step == 0:
					downsampled_ranges.data.append(scan.ranges[i])
					a.insert(0, scan.ranges[i])
				else:
					a.insert(0, 0)
			else:
				a.insert(0, 0)

		# get scan data in deg2 (from 359 to 180)
		for i in reversed(range(180, 360)):
			if deg2.min <= i and i <= deg2.max-1:
				if i % step == 0 and i != deg2.min:
					downsampled_ranges.data.append(scan.ranges[i])
					b.insert(0, scan.ranges[i])
				else:
					b.insert(0, 0)
			else:
				b.insert(0, 0)

		# By using two lists, get a list in which data is stored in the order of 0 to 360 degrees
		a.extend(b)
		downsampled_scan.ranges = a
			
		# publish
 		self.pub_scan_downsampled_forward.publish(downsampled_scan)
		self.pub_ranges_downsampled_forward.publish(downsampled_ranges)
		
if __name__ ==  '__main__':
	try:
		pdss = pubDownSampledScan()
		rospy.spin()
	except rospy.ROSInterruptException: pass
