#!/usr/bin/env python
# This final piece fo skeleton code will be centred around gettign the students to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from move_to_xy import move_base


class CircleIdentifier():

	def __init__(self):

		#global forward
		self.bridge=CvBridge()

		#self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
		self.rate = rospy.Rate(10) #10hz
		self.desired_velocity = Twist()
		# Initialise any flags that signal a colour has been detected in view
		self.red = False
		#self.blue = False
		self.green = False
		self.green_circle_found = False
		self.red_circle_found = False
		self.i = 0

		# Initialise the value for sensitivity in the colour detection
		self.sensitivity = 5


		# set up a subscriber to the image topic
		rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)


	def callback(self, data):
		self.red_circle_found = False
		self.red = False

		try:
			image=self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError:
			print"Error"

		hsv_red_lower = np.array([0 - self.sensitivity, 100, 100]) #set up upper and lower limits for red and green
		hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
		hsv_green_lower = np.array([60 - self.sensitivity, 50, 50])
		hsv_green_upper = np.array([80 + self.sensitivity, 255, 255])

		# Convert the rgb image into a hsv image
		cv2.namedWindow('Circle_feed')
		cv2.imshow('Circle_feed', image)
		cv2.waitKey(3)
		image_HSV=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		# cv2.imshow('zzz', image_HSV)


		# Filter out everything but particular colours using the cv2.inRange() method

		maskr = cv2.inRange(image_HSV, hsv_red_lower, hsv_red_upper)
		maskg = cv2.inRange(image_HSV, hsv_green_lower, hsv_green_upper)

		mask2 = cv2.bitwise_or(maskr, maskg)
		# cv2.imshow('mask 2', mask2)
		# You can only bitwise_or two image at once, so multiple calls are necessary for more than two colours

		# Apply the mask to the original image using the cv2.bitwise_and() method
		image_mask2 = cv2.bitwise_and(image, image, mask = mask2)

		cntsr, hierarchyr = cv2.findContours(maskr.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #intialise contours
		cntsg, hierarchyg = cv2.findContours(maskg.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		cnts, hierarchy = cv2.findContours(mask2.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)


		# Loop over the contours
		if cnts:
			if cv2.contourArea(max(cnts, key=cv2.contourArea)) > 1000:
				if cntsg:
					c = max(cntsg, key=cv2.contourArea)
					M = cv2.moments(c)
					((x, y), self.radius) = cv2.minEnclosingCircle(c)
					cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
					if cv2.contourArea(c) > 1000:#threshold for contour area and selecting green circle
						#self.red = True
						self.green = True
						cv2.circle(image_mask2,(cx,cy), int(self.radius),(255,0,0),1)
						# draw a circle on the contour
					else:
						cv2.circle(image_mask2,(cx,cy), int(self.radius),(0,255,0),1)
						# draw a circle on the contour in a different colour
					height, length = image.shape[:2]
					image_center = (height/2, length/2)

					if self.green == True: # centering on green circle
						if cv2.contourArea(c) > 10500 or cv2.contourArea(c) < 9500:
							if (cx > image_center[0]+20):
								self.desired_velocity.angular.z = -0.1
							elif (cx < image_center[0]-20):
								self.desired_velocity.angular.z = 0.1
							else:
								self.desired_velocity.angular.z = 0
								self.green_circle_found = True
							#	rospy.loginfo('Flag True')
								cv2.imwrite('green_circle.png', image)
								rospy.loginfo('pic taken')

				elif cntsr: # for detecting red circle
					c = max(cntsr, key=cv2.contourArea)
					M = cv2.moments(c)
					((x, y), self.radius) = cv2.minEnclosingCircle(c)
					cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

					if cv2.contourArea(c) > 1000:#<What do you think is a suitable area?>:
						self.red = True
					cv2.circle(image_mask2,(cx,cy), int(self.radius),(0,255,0),1)

					if self.red == True:
						if cv2.contourArea(c) > 10500 or cv2.contourArea(c) < 9500:
							self.desired_velocity.angular.z = 0
							self.red_circle_found = True
						#	rospy.loginfo('RedFlag True')
			else:
				self.desired_velocity.angular.z = 0.2
				self.desired_velocity.linear.x = 0
		else:
			self.desired_velocity.angular.z = 0.2
			self.desired_velocity.linear.x = 0
			#move_base.move_rotate(self, -1)
			

		cv2.imshow('c1', image_mask2)
		cv2.waitKey(3)


	def get_greenflag(self):
		return self.green_circle_found
	def get_redflag(self):
		return self.red_circle_found
	def spin_dir(self):
		return self.desired_velocity.angular.z
# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
# 	# Instantiate your class
# And rospy.init the entire node
	rospy.init_node('CircleIdentifier', anonymous=True)
	cI = CircleIdentifier()
#
	rospy.spin()
# 	# Ensure that the node continues running with rospy.spin()
# 	# You may need to wrap it in an exception handler in case of KeyboardInterrupts
# 	# Remember to destroy all image windows before closing node
#  """
# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
