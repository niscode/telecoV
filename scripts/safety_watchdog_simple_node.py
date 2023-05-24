#!/usr/bin/env python3

import os
import sys
import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalID, GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal
from xmlrpc.client import ServerProxy as XMLServerProxy

PROXIMITY_THRESHOLD = 0.5
COOLDOWN_AFTER_TRIGGER = 5.0
MAX_RETRIES = 5


class SafetyWatchdog:
    def __init__(self) -> None:
        rospy.init_node("safety_watchdog_simple_node")
        self._ros_master = XMLServerProxy(os.environ['ROS_MASTER_URI'])
        self._rosnode_dynamically_loaded = __import__('rosnode')

        if not self._wait_for_node('/move_base', 10.0):
            rospy.logerr('move_base node not available')
            sys.exit(0)

        rospy.Subscriber('/scan', LaserScan, self._laser_scan_cb, queue_size=1)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self._move_base_status_cb, queue_size=1)
        self._navigation_cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self._patrol_cancel_service = rospy.ServiceProxy('/patrol/cancel', Empty)

        self._current_status = GoalStatus.SUCCEEDED
        self._latest_goal = MoveBaseActionGoal()
        self._minimum_laser_range = -1.0
        self._current_retry = 1
        self._goal_header_id_during_incident = 0

        rospy.loginfo('Safety_Watchdog started')

    def _wait_for_node(self, node_name: str, timeout: float) -> bool:
        start_time = rospy.Time.now().to_sec()
        while start_time + timeout >= rospy.Time.now().to_sec():
            try:
                if node_name in self._rosnode_dynamically_loaded.get_node_names():
                    return True
            except self._rosnode_dynamically_loaded.ROSNodeIOException:
                return False
        return False

    def _move_base_status_cb(self, msg: GoalStatusArray) -> None:
        if msg.status_list:
            self._current_status = msg.status_list[-1].status

    def _laser_scan_cb(self, msg: LaserScan) -> None:
        self._minimum_laser_range = min(msg.ranges)

    def _move_base_goal_cb(self, msg: MoveBaseActionGoal) -> None:
        self._latest_goal = msg

    def _cancel_navigation(self) -> None:
        rospy.loginfo(f'Obstacle appeared in a distance of {self._minimum_laser_range}... Canceling navigation...')
        empty_goal = GoalID()
        for i in range(5):
            self._navigation_cancel_publisher.publish(empty_goal)

    def run(self) -> None:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._current_status == GoalStatus.ACTIVE and self._minimum_laser_range < PROXIMITY_THRESHOLD:
                self._patrol_cancel_service()
                self._cancel_navigation()
            rate.sleep()


if __name__ == '__main__':
    try:
        sw = SafetyWatchdog()
        sw.run()
    except rospy.ROSInterruptException:
        pass
