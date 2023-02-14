#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseActionGoal

def callback(data):
    pos = data.goal.target_pose.pose
    print ("[({0},{1},0.0),(0.0,0.0,{2},{3})],".format(pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w))

def listener():

    rospy.init_node('multigoal_getter', anonymous=True)

    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()