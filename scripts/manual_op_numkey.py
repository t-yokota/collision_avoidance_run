#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import sys, select, termios, tty, time
# from geometry_msgs.msg import Twist

msg = """
press 'l' -> linear  velocity setting mode : press '0' ~ '9' (val of velocity)
press 'a' -> angular velocity setting mode : press '0' ~ '9' (val of velocity)
"""

e = """
Communications Failed
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ ==  '__main__':
	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('manual_op_numkey')
	pub_lin_vel = rospy.Publisher('/vel/lin', Float32, queue_size=10)
	pub_ang_vel = rospy.Publisher('/vel/ang', Float32, queue_size=10)

	status = 0

	try:
		print msg
		while(1):
			key = getKey()
			if key == 'l':
				print 'set linear velocity'
				while(not(key.isdigit())):
					key = getKey()
				print '>', key
				linear_vel = Float32()
				linear_vel.data = int(key)
				pub_lin_vel.publish(linear_vel)
				status = status + 1
				print 'pub /vel/lin'
				print '----------'

			elif key == 'a':
				print 'set angular velocity'
				while(not(key.isdigit())):
					key = getKey()
				print '>', key
				angular_vel = Float32()
				angular_vel.data = int(key)
				pub_ang_vel.publish(angular_vel)
				status = status + 1
				print 'pub /vel/ang'
				print '----------'

			else:
				if key == '\x03':
					break

			if status == 5:
				status = 0
				print msg

	except:
		print e

	finally:
		pass

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
