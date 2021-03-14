#!/usr/bin/env python

# Source https://github.com/atomoclast/ar_tag_toolbox
# By Andred Dandough and Varun Agrawal
# Modified for Robaka by Alex Makarov

import rospy
import actionlib
from actionlib_msgs.msg import GoalID
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import copysign
import tf
from tf import transformations as tft
import numpy as np
from math import cos, sin, pi

class ARFollower():
    def __init__(self):
        rospy.init_node("ar_follower")
                        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # How often should we update the robot's motion?
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate) 
        
        # Publisher to control the robot's movement
        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.simple_pub=rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1) 
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        
        # Set flag to indicate when the AR marker is visible
        self.target_visible = False
	self.goal_locked = False

        self.listener=tf.TransformListener()

        # Wait for the ar_pose_marker topic to become available
        rospy.loginfo("Waiting for ar_pose_marker topic...")
        rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
        
        # Subscribe to the ar_pose_marker topic to get the image width and height
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_callback)
        # Begin the cmd_vel publishing loop
        while not rospy.is_shutdown():
            # Sleep for 1/self.rate seconds
            r.sleep()

    def marker_callback(self, msg):
#        if self.goal_locked:
#            return

        # Pick off the first marker (in case there is more than one)
        try:
            marker = msg.markers[0]
            if not self.target_visible:
                rospy.loginfo("FOLLOWER is Tracking Target!")
            self.target_visible = True
            self.goal_locked = True
        except:
            # If target is lost, cancel all goals
            self.move_base.cancel_all_goals()
            self.goal_locked = False
            
            if self.target_visible:
                rospy.loginfo("FOLLOWER LOST Target!")
            self.target_visible = False
            
            return

        # We only care for one marker for now
        frame_id = "ar_marker_0"

        try:
            now=rospy.Time.now()
            self.listener.waitForTransform('map', frame_id , now, rospy.Duration(10))
            trans, rot=self.listener.lookupTransform('map', frame_id, now)

            g = MoveBaseGoal()
            g.target_pose.header.frame_id = 'map'
            g.target_pose.header.stamp=rospy.Time.now()
            x=trans[0]-0.50
            y=trans[1]
            z=trans[2]

            rpy = tft.euler_from_quaternion(rot)
            g.target_pose.pose.orientation = Quaternion(*tft.quaternion_from_euler(0, 0, rpy[2]-3.14159/2.0))
            g.target_pose.pose.position.x = x
            g.target_pose.pose.position.y = y
            g.target_pose.pose.position.z = z
            self.move_base.send_goal(g)

            success_to_marker=self.move_base.wait_for_result()
            if success_to_marker:
                print "Movement to marker successful"
            else:
                print "Movement to marker failed"

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
            print "TF Exception, try again"
            
    def shutdown(self):
        rospy.loginfo("Canceling all goals")
        self.move_base.cancel_all_goals()        
        rospy.sleep(1)     

if __name__ == '__main__':
    try:
        ARFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AR follower node terminated.")
