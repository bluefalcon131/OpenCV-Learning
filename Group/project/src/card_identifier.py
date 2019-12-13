#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CardIdentifier():
	def __init__(self):
		# Initialise a publisher to publish messages to the robot base
		# We covered which topic receives messages that move the robot in the 2nd Lab Session
		move_pub = rospy.Publisher('mobile_base/commands/velocity', Twist)

		# Initialise any flags that signal a colour has been detected in view
		self.found = False

		# Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)

		self.sensitivity = 10

		# Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
		stop_velocity = Twist()
		stop_velocity.linear.x = 0
		stop_velocity.linear.y = 0
		stop_velocity.linear.z = 0

		stop_velocity.angular.x = 0
		stop_velocity.angular.y = 0
		stop_velocity.angular.z = 0

		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		self.bridge = CvBridge()

		# We covered which topic to subscribe to should you wish to receive image data
		rospy.Subscriber('camera/rgb/image_raw', Image, self.card_callback)

	def card_callback(self, data):
		# Convert the received image into a opencv image
		# But remember that you should always wrap a call to this conversion method in an exception handler
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		# Set the upper and lower bounds for the two colours you wish to identify

		hsv_black_lower = np.array([0, 0, 0])
		hsv_black_upper = np.array([255, 255, 50])

		hsv_scarlet_lower = np.array([0 - 5, 100, 100])	# Sensitivity adjusted because scarlet is more similar to cones compared to everything else
		hsv_scarlet_upper = np.array([0 + 5, 255, 255])	# Sensitivity adjusted because scarlet is more similar to cones compared to everything else

		hsv_mustard_lower = np.array([30 - self.sensitivity, 20, 40])
		hsv_mustard_upper = np.array([30 + self.sensitivity, 180, 180])

		hsv_peacock_lower = np.array([100 - self.sensitivity, 100, 100])
		hsv_peacock_upper = np.array([100 + self.sensitivity, 255, 255])

		hsv_plum_lower = np.array([160 - self.sensitivity, 20, 20])
		hsv_plum_upper = np.array([160 + self.sensitivity, 255, 255])

		# Convert the rgb image into a hsv image
		hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

		# Filter out everything but particular colours using the cv2.inRange() method

		black_mask = cv2.inRange(hsv_image, hsv_black_lower, hsv_black_upper)
		# cv2.namedWindow('black')
		# cv2.imshow('black', black_mask)

		scarlet_mask = cv2.inRange(hsv_image, hsv_scarlet_lower, hsv_scarlet_upper)
		mustard_mask = cv2.inRange(hsv_image, hsv_mustard_lower, hsv_mustard_upper)
		peacock_mask = cv2.inRange(hsv_image, hsv_peacock_lower, hsv_peacock_upper)
		plum_mask = cv2.inRange(hsv_image, hsv_plum_lower, hsv_plum_upper)

		# Find the contours that appear within the certain colours mask using the cv2.findContours() method
		black_contour, hierarchy = cv2.findContours(black_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		scarlet_contour, hierarchy = cv2.findContours(scarlet_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		mustard_contour, hierarchy = cv2.findContours(mustard_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		peacock_contour, hierarchy = cv2.findContours(peacock_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		plum_contour, hierarchy = cv2.findContours(plum_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

		# Arrange the contours in an array of groups of contours to index each person.
		contour_array = [scarlet_contour, mustard_contour, peacock_contour, plum_contour]

		hsv_filtered_image = np.zeros(hsv_image.shape[:2], np.uint8)

		self.index = -1
		self.found = False
		# if black_contour:
		# 	card_contour = max(black_contour, key=cv2.contourArea)
		# 	self.found = True
		# 	cv2.drawContours(hsv_filtered_image, [card_contour], 0, (255, 255, 0), 2)

		while (self.index < 3) and (not self.found):
			self.index += 1
			self.previous_area = 0
			for each_contour in contour_array[self.index]:
				hull = cv2.convexHull(each_contour)
				perimeter = cv2.arcLength(hull, True)
				approx_contour = cv2.approxPolyDP(hull, 0.04 * perimeter, True)
				if len(approx_contour) == 4: # checks if the contour has 4 corners
					(x, y, w, h) = cv2.boundingRect(approx_contour)
					ar = float(w) / float(h) # aspect ratio calc
					self.card_area = cv2.contourArea(approx_contour)
					if  (self.card_area > 200) and (self.card_area > self.previous_area):
						cv2.drawContours(hsv_filtered_image, [hull], 0, (255, 255, 0), 2)
						# rospy.loginfo("[Card Identifier] Ar = " + str(ar))
						if (ar < 0.90): # checks if width to length ratio is correct
							detected_card_contour = hull
							M_card = cv2.moments(detected_card_contour)
							self.cx_card, self.cy_card = int(M_card['m10']/M_card['m00']), int(M_card['m01']/M_card['m00'])
							mask = np.zeros(hsv_image.shape[:2], np.uint8)
							cv2.drawContours(mask, [hull], -1, 255, -1)

							hsv_filtered_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
							cv2.drawContours(hsv_filtered_image, [hull], 0, (255, 0, 255), 2)
							cv2.drawContours(hsv_filtered_image, [approx_contour], 0, (0, 255, 0), 4)
							self.found = True
							self.previous_area = self.card_area

			# if self.found:
			# 	rospy.loginfo('[Card Identifier] ' + names[self.index] +' Detected')


		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.
		# cv2.namedWindow('Camera_Feed')
		# cv2.imshow('Camera_Feed', self.cv_image)

		cv2.namedWindow('Filtered_Camera_Feed')
		cv2.imshow('Filtered_Camera_Feed', hsv_filtered_image)
		cv2.waitKey(3)

	# Accessors
	def card_found(self):
		return self.found
	def card_size(self):
		return self.card_area
	def card_position(self):
		return [self.cx_card, self.cy_card]
		# if string == 'x':
		# 	return cx_card
		# else:
		# 	return cy_card

	# Functions
	def declare_card(self):
		names = ["Miss Scarlet", "Colonel Mustard", "Mrs Peacock", "Professor Plum"]
		rospy.loginfo(names[self.index] + ' is detected, taking picture.')
		cv2.imwrite((names[self.index] + '.png'), self.cv_image)

	def __del__(self):
		rospy.loginfo('card identifier shut down')

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such

def main(args):
	# Instantiate your class
	# And rospy.init the entire node
	rospy.init_node('image_listener', anonymous=True)
	cI = CardIdentifier()
	# Ensure that the node continues running with rospy.spin()
	# You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
	try:
		rospy.spin()
	except KeyboardInterrupts as e:
		print (e)
	# Remember to destroy all image windows before closing node
	cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
