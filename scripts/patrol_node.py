#!/usr/bin/env python3

import os
import random
import rospy
import actionlib
from std_srvs.srv import Empty, EmptyResponse
from telecoV.srv import PatrolService, PatrolServiceResponse
from actionlib_msgs.msg import GoalID, GoalStatusArray, GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from telecoV.msg import WaypointArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from xmlrpc.client import ServerProxy as XMLServerProxy


class PatrolServer:
    def __init__(self) -> None:
        rospy.init_node("patrol_node")
        self._ros_master = XMLServerProxy(os.environ['ROS_MASTER_URI'])
        self._rosnode_dynamically_loaded = __import__('rosnode')

        rospy.loginfo('Waiting for move_base')
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()

        rospy.Subscriber('/waypoint_server/waypoints', WaypointArray, self._waypoint_cb, queue_size=1)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self._move_base_status_cb, queue_size=1)
        self._navigation_cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self._patrol_cancel_service = rospy.Service('/patrol/cancel', Empty, self._service_cancel_cb)
        self._patrol_start_service = rospy.Service('/patrol/start', PatrolService, self._service_start_cb)

        self._current_goal_status = GoalStatus.SUCCEEDED
        self._latest_waypoints = {}
        self._latest_goal = PoseStamped()

        self._patrolling = False
        self._requested_waypoints = None
        self._current_index = 0

        rospy.loginfo('PatrolServer started')

    def _move_base_status_cb(self, msg: GoalStatusArray) -> None:
        if msg.status_list:
            self._current_goal_status = msg.status_list[-1].status

    def _waypoint_cb(self, msg: WaypointArray) -> None:
        for w in msg.waypoints:
            label = w.label
            pose = w.pose
            self._latest_waypoints[label] = pose

    def _service_cancel_cb(self, msg: Empty) -> EmptyResponse:
        self._patrolling = False
        rospy.loginfo('Patrol cancelled... Robot will stop after reaching waypoint')
        return EmptyResponse()

    def _service_start_cb(self, msg: PatrolService) -> PatrolServiceResponse:
        if (len(msg.waypoints) == 1 and msg.waypoints[0] == '') or len(msg.waypoints) == 0:
            rospy.loginfo('Received empty patrol call... Patrolling randomly over all known waypoint')
            self._requested_waypoints = None
            self._patrolling = True
        elif (len(msg.waypoints) == 1 and msg.waypoints[0] != '') and msg.waypoints[0] in self._latest_waypoints:
            rospy.loginfo('Received patrol call with only one waypoint... Navigating to waypoint and ending patrol')
            self._send_navigation_goal(self._latest_waypoints[msg.waypoints[0]])
            self._patrolling = False
        else:
            rospy.loginfo('Received patrol call with list of waypoints... Patrolling over list')
            self._requested_waypoints = msg.waypoints
            self._current_index = 0
            self._patrolling = True
        return PatrolServiceResponse()

    def _cancel_navigation(self) -> None:
        empty_goal = GoalID()
        for i in range(5):
            self._navigation_cancel_publisher.publish(empty_goal)

    def _send_navigation_goal(self, pose) -> None:
        goal = MoveBaseGoal()
        goal.target_pose = pose
        goal.target_pose.header.stamp = rospy.Time.now()
        self._move_base_client.send_goal(goal)

    def _send_random_goal(self) -> None:
        label, pose = random.choice(list(self._latest_waypoints.items()))
        while pose == self._latest_goal:
            label, pose = random.choice(list(self._latest_waypoints.items()))
        rospy.loginfo(f'Randomly selected waypoint: {label} as next goal')
        self._send_navigation_goal(pose)

    def _send_next_goal(self) -> None:
        label = self._requested_waypoints[self._current_index % len(self._requested_waypoints)]
        rospy.loginfo(f'Selected requested waypoint: {label} as next goal')
        self._current_index += 1
        self._send_navigation_goal(self._latest_waypoints[label])

    def run(self) -> None:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._patrolling and self._latest_waypoints:
                if self._current_goal_status == GoalStatus.SUCCEEDED:
                    if self._requested_waypoints:
                        self._send_next_goal()
                    else:
                        self._send_random_goal()
                    self._move_base_client.wait_for_result()
                elif self._current_goal_status == GoalStatus.PREEMPTED:
                    rospy.loginfo('Navigation was canceled by user - Patrol Stopped! - Navigate to safe position and restart patrol.')
                    self._patrolling = False
            else:
                rospy.sleep(1.0)
            rate.sleep()


if __name__ == '__main__':
    try:
        ps = PatrolServer()
        ps.run()
    except rospy.ROSInterruptException:
        pass
