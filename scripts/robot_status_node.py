#!/usr/bin/env python3

import rospy
import actionlib
import tf2_py
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from telecoV.msg import StringArray, RobotStatus
import tf2_ros

REFERENCE_FRAME = 'map'
ROBOT_FRAME = 'base_link'
GOAL_STATUS_MAP ={0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED', 3: 'SUCCEEDED', 4: 'ABORTED', 5: 'REJECTED',
                  6: 'PREEMPTING', 7: 'RECALLING', 8: 'RECALLED', 9: 'LOST'}


class RobotStatusNode:
    def __init__(self) -> None:
        rospy.init_node("robot_status_node")

        rospy.loginfo('Waiting for move_base')
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()
        rospy.Subscriber('/move_base/status', GoalStatusArray, self._move_base_status_cb, queue_size=1)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self._move_base_goal_cb, queue_size=1)
        rospy.Subscriber('/robot_status/console_msg', StringArray, self._console_msgs_cb, queue_size=1)
        self._robot_status_publisher = rospy.Publisher('/robot_status/status', RobotStatus, queue_size=10)

        self._reference_frame = rospy.get_param('/waypoint_server/reference_frame', REFERENCE_FRAME)
        self._robot_frame = rospy.get_param('/waypoint_server/robot_frame', ROBOT_FRAME)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._latest_status = GoalStatus.LOST
        self._latest_status_text = ''
        self._latest_goal = PoseStamped()
        self._latest_console_msgs = list()
        rospy.loginfo('Robot_Status started')

    def _move_base_status_cb(self, msg: GoalStatusArray) -> None:
        if msg.status_list:
            self._latest_status = msg.status_list[-1].status
            self._latest_status_text = msg.status_list[-1].text

    def _move_base_goal_cb(self, msg: MoveBaseActionGoal) -> None:
        self._latest_goal = msg.goal.target_pose

    def _console_msgs_cb(self, msg: StringArray) -> None:
        self._latest_console_msgs.extend(msg.data)

    def _get_current_position(self) -> PoseStamped:
        robot_pose = PoseStamped()
        try:
            robot_transform = self._tf_buffer.lookup_transform(self._reference_frame, self._robot_frame, rospy.Time())
            robot_pose = PoseStamped(Header(), Pose(Point(robot_transform.transform.translation.x, robot_transform.transform.translation.y,
                                                          robot_transform.transform.translation.z),
                                                    Quaternion(robot_transform.transform.rotation.x, robot_transform.transform.rotation.y,
                                                               robot_transform.transform.rotation.z, robot_transform.transform.rotation.w)))
            robot_pose.header.frame_id = self._reference_frame
        except tf2_py.LookupException:
            pass
        return robot_pose

    def run(self) -> None:
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rs_msg = RobotStatus()
            rs_msg.header.stamp = rospy.Time.now()
            rs_msg.navigation_status = GOAL_STATUS_MAP[self._latest_status]
            rs_msg.navigation_status_text = self._latest_status_text
            rs_msg.latest_goal = self._latest_goal
            rs_msg.current_position = self._get_current_position()
            rs_msg.console_messages = self._latest_console_msgs
            self._robot_status_publisher.publish(rs_msg)
            self._latest_console_msgs = list()
            rate.sleep()


if __name__ == '__main__':
    try:
        rsn = RobotStatusNode()
        rsn.run()
    except rospy.ROSInterruptException:
        pass
