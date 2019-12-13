#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

hit = False

def callback(data):
    global hit
    if (data.state == 1):
        hit = True
    if (data.state == 0):
        hit = False

def publisher():
    global hit
    rospy.init_node('Walker', anonymous=True)
    sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, callback)
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
    rate = rospy.Rate(10) #10hz
    desired_velocity = Twist()
    while not rospy.is_shutdown():
        # go straight
        desired_velocity.linear.x = 0.4 # Forward with 0.4 m/sec.
        desired_velocity.angular.z = 0 # Halt the robot angular movement.
        for i in range(27):
            if hit:
                break # skip going straight and proceed to stop
            pub.publish(desired_velocity)
            rate.sleep()
        #stop
        desired_velocity.linear.x = 0 # Halt the robot linear movement.
        desired_velocity.angular.z = 0 # Halt the robot angular movement.
        for i in range(10):
            pub.publish(desired_velocity)
            rate.sleep()
        #turn
        desired_velocity.linear.x = 0 # Halt the robot linear movement.
        desired_velocity.angular.z = math.pi/8 # Turn at 1/8 pi radians/sec.
        for i in range(41):
            if hit:
                rospy.loginfo("hit")
                break # skip turning and proceed to stop
            pub.publish(desired_velocity)
            rate.sleep()
        #stop
        desired_velocity.linear.x = 0 # Halt the robot linear movement.
        desired_velocity.angular.z = 0 # Halt the robot angular movement.
        for i in range(10):
            pub.publish(desired_velocity)
            rate.sleep()

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
