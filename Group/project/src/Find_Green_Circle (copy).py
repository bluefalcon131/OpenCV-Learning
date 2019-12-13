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
		rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)


		# Initialise a publisher to publish messages to the robot base
		# We covered which topic receives messages that move the robot in the 2nd Lab Session
		self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
		self.rate = rospy.Rate(10) #10hz
		self.desired_velocity = Twist()

		# Initialise any flags that signal a colour has been detected in view
		self.red = False
		#self.blue = False
		self.green = False
		self.green_circle_found = False
		self.i = 0

		# Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
		self.sensitivity = 10

		# Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)

		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
		# We covered which topic to subscribe to should you wish to receive image data

	def callback(self, data):

		try:
			image=self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError:
			print"Error"
		# Convert the received image into a opencv image
		# But remember that you should always wrap a call to this conversion method in an exception handler


		# Set the upper and lower bounds for the two colours you wish to identify
		hsv_red_lower = np.array([0 - self.sensitivity, 50, 50])
		hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
		hsv_green_lower = np.array([60 - self.sensitivity, 50, 50])
		hsv_green_upper = np.array([80 + self.sensitivity, 255, 255])

		# Convert the rgb image into a hsv image
		cv2.namedWindow('Camera_Feed')
		cv2.imshow('Camera_Feed', image)
		cv2.waitKey(3)
		image_HSV=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		cv2.imshow('HSV_image', image_HSV)

		# Filter out everything but particular colours using the cv2.inRange() method
		# Be sure to do this for the second or third colour as well
		maskr = cv2.inRange(image_HSV, hsv_red_lower, hsv_red_upper)
		maskg = cv2.inRange(image_HSV, hsv_green_lower, hsv_green_upper)
		# To combine the masks you should use the cv2.bitwise_or() method
		mask2 = cv2.bitwise_or(maskr, maskg)
		# You can only bitwise_or two image at once, so multiple calls are necessary for more than two colours

		# Apply the mask to the original image using the cv2.bitwise_and() method
		image_mask2 = cv2.bitwise_and(image, image, mask = mask2)
		# As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
		# As opposed to performing a bitwise_and on the mask and the image.


		# Find the contours that appear within the certain colours mask using the cv2.findContours() method
		# For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
		cntsr, hierarchyr = cv2.findContours(maskr.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		cntsg, hierarchyg = cv2.findContours(maskg.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

		# Loop over the contours
		if len(cntsg) > 0:

			c = max(cntsg, key=cv2.contourArea)
			#cr = max(cntsr, key=cv2.contourArea)
		# There are a few different methods for identifying which contour is the biggest
		# Loop throguht the list and keep track of which contour is biggest or
		# Use the max() method to find the largest contour
			M = cv2.moments(c)
			((x, y), self.radius) = cv2.minEnclosingCircle(c)
			if not(M == 0):
				cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

				#Check if the area of the shape you want is big enough to be considered
				# If it is then change the flag for that colour to be True(1)
				#colour_max_area = 100
				#global c
			if cv2.contourArea(c) > 1000:#<What do you think is a suitable area?>:
				#self.red = True
				self.green = True
				# draw a circle on the contour you're identifying as a blue object as well
			cv2.circle(image_mask2,(cx,cy), int(self.radius),(0,255,0),1)
		# if len(cntsr) > 0:
		# 	#c = max(cntsb, key=cv2.contourArea)
		# 	cr = max(cntsr, key=cv2.contourArea)
		# # There are a few different methods for identifying which contour is the biggest
		# # Loop throguht the list and keep track of which contour is biggest or
		# # Use the max() method to find the largest contour
		# 	Mr = cv2.moments(cr)
		# 	((x, y), self.radiusr) = cv2.minEnclosingCircle(cr)
		# 	if not(Mr == 0):
		# 		cx, cy = int(Mr['m10']/M['m00']), int(Mr['m01']/Mr['m00'])
		#
		# 		#Check if the area of the shape you want is big enough to be considered
		# 		# If it is then change the flag for that colour to be True(1)
		# 		#colour_max_area = 100
		# 		#global c
		# 		if cv2.contourArea(cr) > 1000:#<What do you think is a suitable area?>:
		# 			#self.red = True
		# 			self.red = True
		# 			#self.blue = False
		# 			# draw a circle on the contour you're identifying as a blue object as well
		# 		cv2.circle(image_mask2,(cx,cy), int(self.radiusr),(0,255,0),1)


				# cv2.circle(<image>, (<center x>,<center y>), <radius>, <colour (rgb tuple)>, <thickness (defaults to 1)>)
				# Then alter the values of any flags

			#Check if a flag has been set for the stop message
			#while (self.i < 10):
			#turn = move_base
			height, length = image.shape[:2]
			image_center = (height/2, length/2)
			#self.desired_velocity.angular.z = 0.2
			#self.desired_velocity.linear.x = 0
			#cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			if self.green == True:
				#if not(M == 0):
				if cv2.contourArea(c) > 10500 or cv2.contourArea(c) < 9500:
					if (cx > image_center[0]+20):
						self.desired_velocity.angular.z = -0.1
					elif (cx < image_center[0]-20):
						self.desired_velocity.angular.z = 0.1
					else:
						self.desired_velocity.angular.z = 0
						cv2.imwrite('green_circle.png', image)
						self.green_circle_found = True
				#self.desired_velocity.angular.z = 0
				#self.desired_velocity.linear.x = 0


					#	self.i +=1


		if self.green == False:
			#self.green_circle_found = False
		#	for i in range(30):

			self.desired_velocity.angular.z = 0.2
			self.desired_velocity.linear.x = 0
			#move_base.move_rotate(self, -1)
				#if cv2.contourArea(c) > 10500 or cv2.contourArea(c) < 9500:


					#return

			#	return
					#

			# if self.green == True:
			# 	#global c
			# 	height, length = image.shape[:2]
			# 	image_center = (height/2, length/2)
			# 	if cv2.contourArea(c) > 10500 or cv2.contourArea(c) < 9500 or self.desired_velocity.angular.z == 0: #self.radius < 70 or self.radius == 70:
			# 		if self.desired_velocity.linear.x < 0.2 or self.desired_velocity.linear.x == 0.2:
			# 			self.desired_velocity.linear.x = -(cv2.contourArea(c) - 10000)/10000
			# 		else:
			# 			self.desired_velocity.linear.x = 0.2
			# 		#for i in range(60):
			# 		# if (cv2.contourArea(c)) < 10000:
			# 		#
			# 		# 	self.desired_velocity.linear.x = 0.2 # Forward with 0.2 m/sec.
			# 		#
			# 		# 	# Too close to object, need to move backwards
			# 		# 	# linear = positive
			# 		# 	# angular = radius of minimum enclosing circle
			# 		# #elif (colour_max_area) < 9000:
			# 		# elif (cv2.contourArea(c)) > 10000:
			# 		# 	#self.desired_velocity.angular.z = 0
			# 		# 	self.desired_velocity.linear.x =-0.2 # Forward with 0.2 m/sec.
			# 		# else:
			# 		# 	self.desired_velocity.linear.x = 0
			# 		if (cx > image_center[0]):
			# 			self.desired_velocity.angular.z = -0.1
			# 		elif (cx < image_center[0]):
			# 			self.desired_velocity.angular.z = 0.1
			# 		else:
			# 			self.desired_velocity.angular.z = 0
			# 			#self.desired_velocity.angular.z = 0
			# 			#self.desired_velocity.linear.x = 0
			#
			# 	else:
			# 		# for i in range(10):
			# 		cv2.imwrite('green_circle.png', image)
			# 		for i in range(30):
			# 			self.desired_velocity.angular.z = 0
			# 			self.desired_velocity.linear.x = 0
			# 		#cv2.waitKey(3)
			# 		if self.green == False:
			# 			for i in range(40):
			# 				self.desired_velocity.angular.z = 0
			# 				self.desired_velocity.linear.x = 0.2
			# 			self.desired_velocity.angular.z = 0
			# 			self.desired_velocity.linear.x = 0
			# 		else:
			# 			self.desired_velocity.angular.z = 0.2




			# if self.red == True:
			# #
			# #
			# 	self.desired_velocity.linear.x = 0 # Forward with 0.2 m/sec.
			# #
			# #
			# 	self.desired_velocity.angular.z = 0
			# 	return
			#
			# 	# self.<publisher_name>.publish(<Move>)
			#self.pub.publish(self.desired_velocity)
			#rate.sleep()
		# Be sure to do this for the other colour as well
		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.
		self.pub.publish(self.desired_velocity)
		cv2.imshow('c1', image_mask2)
		cv2.waitKey(3)
# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
	# Instantiate your class
	# And rospy.init the entire node
	rospy.init_node('CircleIdentifier', anonymous=True)
	cI = CircleIdentifier()

	rospy.spin()
	# Ensure that the node continues running with rospy.spin()
	# You may need to wrap it in an exception handler in case of KeyboardInterrupts
	# Remember to destroy all image windows before closing node

# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
