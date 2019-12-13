#!/usr/bin/env python

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
		# Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
		global sensitivity
		sensitivity = 10
		# Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		self.bridge = CvBridge()
		# We covered which topic to subscribe to should you wish to receive image data
		rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)

	def callback(self, data):
		# Convert the received image into a opencv image
		# But remember that you should always wrap a call to this conversion method in an exception handler
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		# Set the upper and lower bounds for the two colours you wish to identify
		global sensitivity
		hsv_red_lower = np.array([0 - sensitivity, 100, 100])
		hsv_red_upper = np.array([0 + sensitivity, 255, 255])

		hsv_green_lower = np.array([60 - sensitivity, 100, 100])
		hsv_green_upper = np.array([60 + sensitivity, 255, 255])

		hsv_blue_lower = np.array([120 - sensitivity, 100, 100])
		hsv_blue_upper = np.array([120 + sensitivity, 255, 255])


		# Convert the rgb image into a hsv image
		hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

		# Filter out everything but particular colours using the cv2.inRange() method
		red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
		green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
		blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

		# To combine the masks you should use the cv2.bitwise_or() method
		# You can only bitwise_or two image at once, so multiple calls are necessary for more than two colours
		mask = cv2.bitwise_or(red_mask, green_mask)
		mask = cv2.bitwise_or(mask, blue_mask)

		# Apply the mask to the original image using the cv2.bitwise_and() method
		hsv_filtered_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

		# As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
		# As opposed to performing a bitwise_and on the mask and the image.

		#Show the resultant images you have created. You can show all of them or just the end result if you wish to.
		cv2.namedWindow('Camera_Feed')
		cv2.imshow('Camera_Feed', cv_image)

		cv2.namedWindow('Filtered_Camera_Feed')
		cv2.imshow('Filtered_Camera_Feed', hsv_filtered_image)
		cv2.waitKey(3)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
	# Instantiate your class
	# And rospy.init the entire node
	rospy.init_node('image_listener', anonymous=True)
	cI = colourIdentifier()
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
