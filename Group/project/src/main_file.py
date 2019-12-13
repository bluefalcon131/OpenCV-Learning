#!/usr/bin/env python
#main_file
import math
import cv2
from move_to_xy import move_base
# from Find_Green_Circle import CircleIdentifier
# from get_coords import get_coords
from card_identifier import CardIdentifier
from get_coords import get_coords
import rospy
from Find_Green_Circle2 import CircleIdentifier
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
global bump_flag

def main():
	# Instantiate your class
	# And rospy.init the entire node
	rospy.init_node('Cluedo_main', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	# Initialise all classes
	global bump_flag
	bump_flag = False
	rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bump)
	cirI = CircleIdentifier()
	cI = CardIdentifier()
	nav = move_base()
	room1_entrance,room1_centre, room2_entrance,room2_centre = get_coords()
	room_entrances = [room1_entrance, room2_entrance]
	room_centres = [[0,0], room1_centre, room2_centre]
	offset = [[0,0],[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0],[-1,1]]
	room = 0

	# Search for green circle
	while (not rospy.is_shutdown()) and (room == 0):
		nav.goto_xy(room1_entrance[0],room1_entrance[1],0)
		for i in range(350):
			nav.move_rotate_speed(cirI.spin_dir()*1000)
			rospy.loginfo("green: " + str(cirI.get_greenflag()))
			rospy.loginfo("red: " + str(cirI.get_redflag()))
			if cirI.get_greenflag() == True:
				room = 1
				rospy.loginfo('I have found room ' + str(room))
				break
			if rospy.is_shutdown() or cirI.get_redflag():
				break
			rate.sleep()

		if room == 0:
			nav.goto_xy(room2_entrance[0],room2_entrance[1],33)
			for i in range(350):
				nav.move_rotate_speed(cirI.spin_dir()*1000)
				rospy.loginfo("green: " + str(cirI.get_greenflag()))
				rospy.loginfo("red: " + str(cirI.get_redflag()))
				if cirI.get_greenflag() == True:
					room = 2
					rospy.loginfo('I have found room ' + str(room))
					break
				if rospy.is_shutdown() or cirI.get_redflag():
					break
				rate.sleep()

	# move_into_room()
	nav.goto_xy(room_centres[room][0],room_centres[room][1],33)
	nav.move_rotate_speed(100)
	# Search for card
	# rospy.loginfo(str(cI.card_found()))
	offsetn = 0
	offsetm = 1
	while (not rospy.is_shutdown()):
		nav.goto_xy(room_centres[room][0]+(offset[offsetn][0]*offsetm),room_centres[room][1]+(offset[offsetn][1]*offsetm),33)
		bump_flag = False
		for i in range(350):
			nav.move_rotate_speed(100)
			rate.sleep()
			if rospy.is_shutdown() or bump_flag:
				break
			elif cI.card_found():
				while not rospy.is_shutdown():		# Center the card
					steer_error = -cI.card_position()[0] + 360
					# rospy.loginfo(str(steer_error))
					nav.move_rotate_speed(steer_error)
					if ((-80 < steer_error) and (steer_error < 80)) or bump_flag:
						# rospy.loginfo('escape')
						break
				while not rospy.is_shutdown():		# Approach the card
					distance_error = -(math.sqrt(cI.card_size()) - 120)
					# rospy.loginfo(str(distance_error))
					steer_error = -cI.card_position()[0] + 320
					# rospy.loginfo(str(steer_error))
					nav.move_steer_speed(steer_error,distance_error*2)

					if (-10 < distance_error) and (distance_error < 10) or bump_flag:
						# rospy.loginfo(str(distance_error))
						break
				nav.move_steer_speed(0,0)	# Stop movement
			# Approach the card
				if not bump_flag:
					cI.declare_card() # take picture and declare card on log
					rospy.loginfo('Program is done, shutting down.')
					rospy.signal_shutdown('Program is done.')
		offsetn +=1
		if offsetn == 6:
			offsetn = 1
			offsetm += 0.3

	# Ensure that the node continues running with rospy.spin()
	# You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
	try:
		rospy.spin()
	except KeyboardInterrupts as e:
		print (e)
	# Remember to destroy all image windows before closing node
	cv2.destroyAllWindows()
def bump(self):
	global bump_flag
	rospy.loginfo("bump")
	bump_flag = True
if __name__ == '__main__':
	main()
