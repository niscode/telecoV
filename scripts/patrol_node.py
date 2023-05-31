#!/usr/bin/env python3

import os
import random
import rospy
import actionlib
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Header
from telecoV.srv import PatrolService, PatrolServiceResponse
from actionlib_msgs.msg import GoalID, GoalStatusArray, GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from telecoV.msg import WaypointArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

REFERENCE_FRAME = 'map'
ROBOT_FRAME = 'base_link'


class PatrolServer:
    def __init__(self) -> None:
        rospy.init_node("patrol_node")

        self._latest_goal_status = GoalStatus.SUCCEEDED
        self._received_waypoints = dict()
        self._latest_goal = PoseStamped()

        self._should_patrol = False
        self._reach_waypoint = False
        self._requested_waypoints = None
        self._current_index = 0

        self._reference_frame = rospy.get_param('/patrol/reference_frame', REFERENCE_FRAME)
        self._robot_frame = rospy.get_param('/patrol/robot_frame', ROBOT_FRAME)

        rospy.loginfo('Waiting for move_base')
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()

        rospy.Subscriber('/waypoint_server/waypoints', WaypointArray, self._waypoint_cb, queue_size=1)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self._move_base_status_cb, queue_size=1)
        self._navigation_cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self._patrol_cancel_service = rospy.Service('/patrol/cancel', Empty, self._service_cancel_cb)
        self._patrol_start_service = rospy.Service('/patrol/start', PatrolService, self._service_start_cb)

        rospy.loginfo('PatrolServer started')

    def _move_base_status_cb(self, msg: GoalStatusArray) -> None:
        if msg.status_list:
            state = msg.status_list[-1].status
            if (state == GoalStatus.PREEMPTED or state == GoalStatus.PREEMPTING) and not (self._latest_goal_status == GoalStatus.PREEMPTED or self._latest_goal_status == GoalStatus.PREEMPTING):
                rospy.loginfo('Navigation was externally preempted - Patrol Stopped!')
                self._should_patrol = False
            if state == GoalStatus.ACTIVE and self._latest_goal_status != state == GoalStatus.ACTIVE and self._latest_goal_status != state == GoalStatus.LOST:
                self._should_patrol = True
            self._latest_goal_status = state

    def _waypoint_cb(self, msg: WaypointArray) -> None:
        latest_waypoints = dict()
        for w in msg.waypoints:
            latest_waypoints[w.label] = w.pose
        self._received_waypoints = latest_waypoints

    def _service_cancel_cb(self, msg: Empty) -> EmptyResponse:
        self._should_patrol = False
        self._reach_waypoint = False
        rospy.loginfo('Patrol cancelled... Robot will stop after reaching current waypoint.')
        return EmptyResponse()

    def _service_start_cb(self, msg: PatrolService) -> PatrolServiceResponse:
        if (len(msg.waypoints) == 1 and msg.waypoints[0] == '') or len(msg.waypoints) == 0:
            rospy.loginfo('Received empty patrol call... Patrolling randomly over all known waypoint')
            self._requested_waypoints = None
            self._should_patrol = True
        elif (len(msg.waypoints) == 1 and msg.waypoints[0] != '') and msg.waypoints[0] in self._received_waypoints:
            rospy.loginfo('Received patrol call with only one waypoint... Navigating to waypoint and ending patrol')
            self._requested_waypoints = msg.waypoints
            self._current_index = 0
            self._should_patrol = False
            self._reach_waypoint = True
        else:
            rospy.loginfo('Received patrol call with list of waypoints... Patrolling over list')
            self._requested_waypoints = msg.waypoints
            self._current_index = 0
            self._should_patrol = True
        return PatrolServiceResponse()

    def _send_navigation_goal(self, pose) -> None:
        goal = MoveBaseGoal()
        goal.target_pose = pose
        goal.target_pose.header.stamp = rospy.Time.now()
        self._latest_goal = pose
        self._move_base_client.send_goal(goal)

    def _send_random_goal(self) -> None:
        label, pose = random.choice(list(self._received_waypoints.items()))
        while pose == self._latest_goal:
            label, pose = random.choice(list(self._received_waypoints.items()))
        rospy.loginfo(f'Randomly selected waypoint: {label} as next goal')
        self._send_navigation_goal(pose)

    def _send_next_goal(self) -> None:
        label = self._requested_waypoints[self._current_index % len(self._requested_waypoints)]
        rospy.loginfo(f'Selected requested waypoint: {label} as next goal')
        self._current_index += 1
        if label in self._received_waypoints:
            self._send_navigation_goal(self._received_waypoints[label])
        else:
            rospy.loginfo(f'Waypoint {label} is unknown and will be ignored')

    def run(self) -> None:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._received_waypoints and self._reach_waypoint:
                self._send_next_goal()
                self._move_base_client.wait_for_result(rospy.Duration(0))
                self._reach_waypoint = False
                self._should_patrol = False
            if self._received_waypoints and self._should_patrol:
                if self._requested_waypoints:
                    self._send_next_goal()
                else:
                    self._send_random_goal()
                self._move_base_client.wait_for_result(rospy.Duration(0))
            rate.sleep()


if __name__ == '__main__':
    try:
        ps = PatrolServer()
        ps.run()
    except rospy.ROSInterruptException:
        pass
