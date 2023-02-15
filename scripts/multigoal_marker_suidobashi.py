#!/usr/bin/env python
import rospy
import csv
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal

# 変数 [waypoints] にmulitgoal_getter.pyで取得したリストを格納
waypoints = [

]

def goal_pose(pose): 
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose


rospy.init_node("multigoal_marker")
pub = rospy.Publisher("waypoint", Marker, queue_size = 10)
rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, goal_pose)
rate = rospy.Rate(25)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        counter = 0
        for i, pose in enumerate(waypoints):
            goal = goal_pose(pose)

            # Mark arrow   矢印を出す
            marker_data = Marker()
            marker_data.header.frame_id = "map"
            marker_data.header.stamp = rospy.Time.now()

            marker_data.ns = "basic_shapes"
            marker_data.id = counter

            marker_data.action = Marker.ADD

            marker_data.pose.position.x = goal.target_pose.pose.position.x
            marker_data.pose.position.y = goal.target_pose.pose.position.y
            marker_data.pose.position.z = goal.target_pose.pose.position.z

            marker_data.pose.orientation.x = goal.target_pose.pose.orientation.x
            marker_data.pose.orientation.y = goal.target_pose.pose.orientation.y
            marker_data.pose.orientation.z = goal.target_pose.pose.orientation.z
            marker_data.pose.orientation.w = goal.target_pose.pose.orientation.w

            marker_data.color.r = 0.290
            marker_data.color.g = 0.509
            marker_data.color.b = 0.490
            marker_data.color.a = 1.0
            marker_data.scale.x = 0.8
            marker_data.scale.y = 0.1
            marker_data.scale.z = 0.02

            marker_data.lifetime = rospy.Duration()

            marker_data.type = 0

            pub.publish(marker_data)
            counter +=1


            # Mark num   数字を添える
            marker_data = Marker()
            marker_data.header.frame_id = "map"
            marker_data.header.stamp = rospy.Time.now()
    
            marker_data.ns = "basic_shapes"
            marker_data.id = counter
            marker_data.action = Marker.ADD

            marker_data.pose.position.x = goal.target_pose.pose.position.x
            marker_data.pose.position.y = goal.target_pose.pose.position.y
            marker_data.pose.position.z = goal.target_pose.pose.position.z

            marker_data.pose.orientation.x = goal.target_pose.pose.orientation.x
            marker_data.pose.orientation.y = goal.target_pose.pose.orientation.y
            marker_data.pose.orientation.z = goal.target_pose.pose.orientation.z
            marker_data.pose.orientation.w = goal.target_pose.pose.orientation.w

            marker_data.color.r = 0.0
            marker_data.color.g = 0.0
            marker_data.color.b = 0.0
            marker_data.color.a = 1.0
            marker_data.scale.x = 1
            marker_data.scale.y = 1
            marker_data.scale.z = 1

            marker_data.lifetime = rospy.Duration()

            marker_data.type = Marker.TEXT_VIEW_FACING
            # marker_data.text = str(int(map(float,row)[0]))
            marker_data.text = str(i+1)

            pub.publish(marker_data)
            counter +=1

        rate.sleep()

    rospy.spin()