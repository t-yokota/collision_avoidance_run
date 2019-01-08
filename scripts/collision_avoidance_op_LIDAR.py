#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

from operator import mul

class pubControlParam_LIDAR:
	def __init__(self):
		rospy.init_node('avoidRun', anonymous=True)
		rospy.Subscriber('/scan/downsampled/forward/ranges', Float32MultiArray, self.subRangeCallback)
		self.pub_lin_vel = rospy.Publisher('/vel/lin', Float32, queue_size=10)
		self.pub_ang_vel = rospy.Publisher('/vel/ang', Float32, queue_size=10)

	def subRangeCallback(self, downsampled_range):
		range_min = 0.1
		range_max = 3.5
		a = list()

		# nomalize
		for i in range(len(downsampled_range.data)):
			if downsampled_range.data[i] == float('inf'):
				a.append(1-(range_max-range_min)/(range_max-range_min))
			else:
				a.append(1-(downsampled_range.data[i]-range_min)/(range_max-range_min))

		for i in range(len(a)):
			rospy.loginfo("val[%d]:%f", i, a[i])

		linVel = 0.5
		angVel = 0.05 * (a[8] + 0.2*(a[7]+a[9]) + 0.1*(a[6]+a[10]) + 0.01*(a[5]+a[11]))
		if (a[0]+a[1]+a[2]+a[3]+a[4]+a[5]+a[6]+a[7]) > (a[9]+a[10]+a[11]+a[12]+a[13]+a[14]+a[15]+a[16]):
			angVel = angVel * -1
		else:
			pass

		self.pub_lin_vel.publish(linVel)
		self.pub_ang_vel.publish(angVel)
		# opval = sum(map(mul, w, a))
		# rospy.loginfo("val:%f", opval)

if __name__ ==  '__main__':
	try:
		pcpl = pubControlParam_LIDAR()
		rospy.spin()
	except rospy.ROSInterruptException: pass