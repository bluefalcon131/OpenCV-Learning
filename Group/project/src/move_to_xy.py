import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Twist, Vector3
import sys, time
from get_coords import get_coords

class move_base():
    def __init__(self):

        self.goal_sent = False
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
        rospy.on_shutdown(self.shutdown)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("[move_to_xy] Wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto_xy(self,x,y,theta): #Move to coordinates and angle
        pos = {'x': x, 'y' : y}
        quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
            Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        self.move_base.send_goal(goal)
        success = self.move_base.wait_for_result(rospy.Duration(60))
        state = self.move_base.get_state()
        result = False
        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def move_rotate(self,r): ##Slightly arbitary rotation r where r > 0 is a clockwise rotation
        vel = Twist()
        if r < 0:
            vel.angular.z = 0.15
        else:
            vel.angular.z = -0.15

        t0 = rospy.Time.now().to_sec()
        current_ang = 0
        while(current_ang < r):
            self.pub.publish(vel)
            t1=rospy.Time.now().to_sec()
            current_ang= 0.15*(t1-t0)
        vel.angular.z = 0
        self.pub.publish(vel)

    def move_rotate_speed(self,speed): ## Rotates the robot with speed of -100 to 100 speed > 0 is a counter-clockwise rotation
        vel = Twist()
        vel.angular.z = float(speed)*0.2/100
        if vel.angular.z > 0.2:    # 0.2m/s limit
            vel.angular.z = 0.2
        elif vel.angular.z < -0.2:    # 0.2m/s limit
            vel.angular.z = -0.2
        self.pub.publish(vel)

    def move_straight(self,v,d): ## move stright with velocity of v for a duration of d
        vel = Twist()
        vel.linear.x = v

        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        while(current_distance < d):
            self.pub.publish(vel)
            t1=rospy.Time.now().to_sec()
            current_distance= v*(t1-t0)
        vel.linear.x = 0
        self.pub.publish(vel)

    def move_steer_speed(self,steer_speed,speed): ## move stright with velocity of v for a duration of d
        vel = Twist()
        vel.angular.z = float(steer_speed)*0.2/100
        vel.linear.x = float(speed)*0.2/100
        if vel.linear.x > 0.2:    # 0.2m/s limit
            vel.linear.x = 0.2
        elif vel.linear.x < -0.2:    # 0.2m/s limit
            vel.linear.x = -0.2
        if vel.angular.z > 0.2:    # 0.2m/s limit
            vel.angular.z = 0.2
        elif vel.angular.z < -0.2:    # 0.2m/s limit
            vel.angular.z = -0.2
        self.pub.publish(vel)


    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=True)
        navigator = move_base()

        room1_entrance,room1_centre, room2_entrance,room2_centre = get_coords()
        # success = navigator.goto_xy(room2_entrance[0],room2_entrance[1],0)
        navigator.move_rotate(0.3)
        # if success:
        #     rospy.loginfo("Hooray, reached the desired pose")
        # else:
        #     rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
