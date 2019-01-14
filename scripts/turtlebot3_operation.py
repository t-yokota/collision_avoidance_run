#!/usr/bin/env python

# editted by tomohiro yokota, 2019

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------

CTRL-C to quit
"""

e = """
Communications Failed
"""

def vels(linear_vel, angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (linear_vel, angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

class robotOperation:
    def __init__(self):
        rospy.init_node('turtlebot3_operation')
        rospy.Subscriber('/vel/lin', Float32, self.opLinCallback)
        rospy.Subscriber('/vel/ang', Float32, self.opAngCallback)
        rospy.Timer(rospy.Duration(0.1), self.smoothCallback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        self.target_linear_vel  = 0.0
        self.control_linear_vel = 0.0

        self.target_angular_vel  = 0.0
        self.control_angular_vel = 0.0

    def opLinCallback(self, linVel):
        self.target_linear_vel  = MAX_LIN_VEL * linVel.data
        self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel)
        # print vels(self.target_linear_vel, self.target_angular_vel)

    def opAngCallback(self, angVel):
        self.target_angular_vel = MAX_ANG_VEL * angVel.data
        self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel)
        # print vels(self.target_linear_vel, self.target_angular_vel)

    def smoothCallback(self, event):
        twist = Twist()
        self.control_linear_vel  = makeSimpleProfile(self.control_linear_vel,  self.target_linear_vel,  (LIN_VEL_STEP_SIZE/2.0))
        self.control_angular_vel = makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))

        twist.linear.x = self.control_linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0 
        twist.angular.z = self.control_angular_vel
        
        self.pub.publish(twist)
        print vels(self.control_linear_vel,self.control_angular_vel)

    def closeCallback(self):
        twist = Twist()
        twist.linear.x  = 0.0; twist.linear.y  = 0.0; twist.linear.z  = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.pub.publish(twist)

if __name__=="__main__":
    print msg
    turtlebot3_model = rospy.get_param("model", "burger")
    try:
        ro = robotOperation()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
    finally:
        pass