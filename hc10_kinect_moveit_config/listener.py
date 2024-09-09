#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import MoveGroupActionGoal

TopicName = "/move_group/goal"
 
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.goal)
     
def listener():
    # Init the listener
    rospy.init_node('listener', anonymous=True)
    
    rospy.Subscriber(TopicName, MoveGroupActionGoal, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()