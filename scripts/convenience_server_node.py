#!/usr/bin/env python3

import os
import sys
import math
import numpy as np
import rospy
import actionlib
from actionlib_msgs.msg import GoalID, GoalStatusArray, GoalStatus
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from telecoV.msg import RobotStatus, WaypointArray
from telecoV.srv import (GoalHeadingService, GoalHeadingServiceResponse, RelativeTurnService, RelativeTurnServiceResponse,
                         GoalHeadingByLabelService, GoalHeadingByLabelServiceResponse)
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler

REFERENCE_FRAME = 'map'
ROBOT_FRAME = 'base_link'


class ConvenienceServer:
    def __init__(self) -> None:
        rospy.init_node("convenience_server_node")

        self._reference_frame = rospy.get_param('/convenience_server/reference_frame', REFERENCE_FRAME)
        self._robot_frame = rospy.get_param('/convenience_server/robot_frame', ROBOT_FRAME)

        rospy.loginfo('Waiting for move_base')
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()

        self._goal_heading_debug_publisher = rospy.Publisher('/convenience_server/debug/goal_heading', PoseStamped, queue_size=10)
        rospy.Subscriber('/robot_status/status', RobotStatus, self._robot_status_cb, queue_size=1)
        rospy.Subscriber('/waypoint_server/waypoints', WaypointArray, self._waypoint_cb, queue_size=1)
        self._goal_heading_service = rospy.Service('/convenience_server/goal_heading', GoalHeadingService, self._goal_heading_service_cb)
        self._goal_heading_by_label_service = rospy.Service('/convenience_server/goal_heading_by_label', GoalHeadingByLabelService, self._goal_heading_by_label_service_cb)
        self._relative_turn_service = rospy.Service('/convenience_server/relative_turn', RelativeTurnService, self._relative_turn_service_cb)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._latest_robot_status = RobotStatus()
        self._requested_waypoints = dict()

        rospy.loginfo('Convenience_Server started')

    def _robot_status_cb(self, msg: RobotStatus) -> None:
        self._latest_robot_status = msg

    def _waypoint_cb(self, msg: WaypointArray) -> None:
        latest_waypoints = dict()
        for w in msg.waypoints:
            latest_waypoints[w.label] = w.pose
        self._received_waypoints = latest_waypoints

    def _relative_turn_service_cb(self, msg: RelativeTurnService) -> RelativeTurnServiceResponse:
        robot_transform = self._tf_buffer.lookup_transform(self._reference_frame, self._robot_frame, rospy.Time())
        _, _, robot_yaw = euler_from_quaternion([robot_transform.transform.rotation.x, robot_transform.transform.rotation.y,
                                                 robot_transform.transform.rotation.z, robot_transform.transform.rotation.w])

        _, _, relative_yaw = euler_from_quaternion([msg.relative_rotation.x, msg.relative_rotation.y, msg.relative_rotation.z,
                                                    msg.relative_rotation.w])

        adjusted_quaternion = quaternion_from_euler(0.0, 0.0, robot_yaw + relative_yaw)

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = PoseStamped(Header(stamp=rospy.Time.now(), frame_id=self._reference_frame),
                                                 Pose(Point(x=robot_transform.transform.translation.x,
                                                            y=robot_transform.transform.translation.y,
                                                            z=robot_transform.transform.translation.z),
                                                      Quaternion(*adjusted_quaternion)))
        self._move_base_client.send_goal(move_base_goal)

        return RelativeTurnServiceResponse(success=True)

    def _goal_heading_service_cb(self, msg: GoalHeadingService) -> GoalHeadingServiceResponse:
        robot_transform = self._tf_buffer.lookup_transform(self._reference_frame, self._robot_frame, rospy.Time())

        robot_vec = np.array([robot_transform.transform.translation.x,
                              robot_transform.transform.translation.y])
        goal_vec = np.array([msg.goal_pose.pose.position.x,
                             msg.goal_pose.pose.position.y])
        difference_vec = goal_vec - robot_vec

        absolute_yaw = math.atan2(difference_vec[1], difference_vec[0])

        _, _, robot_yaw = euler_from_quaternion([robot_transform.transform.rotation.x, robot_transform.transform.rotation.y,
                                                 robot_transform.transform.rotation.z, robot_transform.transform.rotation.w])

        relative_yaw = absolute_yaw - robot_yaw
        goal_heading_quaternion = Quaternion(*quaternion_from_euler(0.0, 0.0, relative_yaw))

        if self._goal_heading_debug_publisher.get_num_connections() > 0:
            debug_pose = PoseStamped()
            debug_pose.header.stamp = rospy.Time.now()
            debug_pose.header.frame_id = self._robot_frame
            debug_pose.pose.position = Point()
            debug_pose.pose.orientation = goal_heading_quaternion
            self._goal_heading_debug_publisher.publish(debug_pose)

        return GoalHeadingServiceResponse(goal_heading_quaternion)

    def _goal_heading_by_label_service_cb(self, msg: GoalHeadingByLabelService) -> GoalHeadingByLabelServiceResponse:
        goal_heading_quaternion = Quaternion(0, 0, 0 ,0)
        success = False
        try:
            goal_pose = self._requested_waypoints[msg.goal_label]
            robot_transform = self._tf_buffer.lookup_transform(self._reference_frame, self._robot_frame, rospy.Time())

            robot_vec = np.array([robot_transform.transform.translation.x,
                                  robot_transform.transform.translation.y])
            goal_vec = np.array([goal_pose.pose.position.x,
                                 goal_pose.pose.position.y])
            difference_vec = goal_vec - robot_vec

            absolute_yaw = math.atan2(difference_vec[1], difference_vec[0])

            _, _, robot_yaw = euler_from_quaternion([robot_transform.transform.rotation.x, robot_transform.transform.rotation.y,
                                                     robot_transform.transform.rotation.z, robot_transform.transform.rotation.w])

            relative_yaw = absolute_yaw - robot_yaw
            goal_heading_quaternion = Quaternion(*quaternion_from_euler(0.0, 0.0, relative_yaw))
            success = True

            if self._goal_heading_debug_publisher.get_num_connections() > 0:
                debug_pose = PoseStamped()
                debug_pose.header.stamp = rospy.Time.now()
                debug_pose.header.frame_id = self._robot_frame
                debug_pose.pose.position = Point()
                debug_pose.pose.orientation = goal_heading_quaternion
                self._goal_heading_debug_publisher.publish(debug_pose)
        except KeyError:
            pass

        return GoalHeadingByLabelServiceResponse(goal_heading=goal_heading_quaternion, success=success)

    def run(self) -> None:
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        cs = ConvenienceServer()
        cs.run()
    except rospy.ROSInterruptException:
        pass
