from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():

    def __init__(self):

        #rospy.init_node('colourIdentifier', anonymous=True)
        self.bridge=CvBridge()
		rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)

        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
		self.rate = rospy.Rate(10) #10hz
		self.desired_velocity = Twist()

        self.green = False

        self.sensitivity = 10

        rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)

    def callback(self, data):

        try:
			image=self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError:
			print"Error"

		hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
		hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

        cv2.namedWindow('Camera_Feed')
		cv2.imshow('Camera_Feed', image)
		cv2.waitKey(3)
		image_HSV=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		cv2.imshow('HSV_image', image_HSV)

        maskg = cv2.inRange(image_HSV, hsv_green_lower, hsv_green_upper)

        image_mask2 = cv2.bitwise_and(image, image, mask = mask2)

        cntsg, hierarchyg = cv2.findContours(maskg.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(cntsg) > 0:

            c = max(cntsg, key=cv2.contourArea)

            M = cv2.moments(c)
			((x, y), self.radius) = cv2.minEnclosingCircle(c)
			if not(M == 0):
				cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

                if cv2.contourArea(c) > 1000:
                    self.green = True

                cv2.circle(image_mask2,(cx,cy), int(self.radius),(0,255,0),1)

        	if self.blue == True:
				#global c
				height, length = image.shape[:2]
				image_center = (height/2, length/2)
				if (cv2.contourArea(c)) < 10000:

					self.desired_velocity.linear.x = 0.2 # Forward with 0.2 m/sec.

				elif (cv2.contourArea(c)) > 10000:
					#self.desired_velocity.angular.z = 0
					self.desired_velocity.linear.x =-0.2 # Forward with 0.2 m/sec.
				else:
					self.desired_velocity.linear.x = 0
				if (cx > image_center[0]):
					self.desired_velocity.angular.z = -0.1
				elif (cx < image_center[0]):
					self.desired_velocity.angular.z = 0.1
				else:
					self.desired_velocity.angular.z = 0

            self.pub.publish(self.desired_velocity)

            cv2.imshow('c1', image_mask2)
    		cv2.waitKey(3)

def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    cI = colourIdentifier()
    rospy.init_node('colourIdentifier', anonymous=True)


    rospy.spin()
	# Ensure that the node continues running with rospy.spin()
	# You may need to wrap it in an exception handler in case of KeyboardInterrupts
	# Remember to destroy all image windows before closing node

# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
