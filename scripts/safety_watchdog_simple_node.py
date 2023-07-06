#!/usr/bin/env python3

import os
import sys
import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalID, GoalStatusArray, GoalStatus
from dynamic_reconfigure.server import Server
from telecoV.cfg import SafetyWatchdogConfig
from telecoV.msg import StringArray
from xmlrpc.client import ServerProxy as XMLServerProxy

PROXIMITY_THRESHOLD = 0.5


class SafetyWatchdog:
    def __init__(self) -> None:
        rospy.init_node("safety_watchdog")
        self._ros_master = XMLServerProxy(os.environ['ROS_MASTER_URI'])
        self._rosnode_dynamically_loaded = __import__('rosnode')

        if not self._wait_for_node('/move_base', 10.0):
            rospy.logerr('move_base node not available')
            sys.exit(0)

        rospy.Subscriber('/scan', LaserScan, self._laser_scan_cb, queue_size=1)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self._move_base_status_cb, queue_size=1)
        self._navigation_cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self._robot_status_msg_publisher = rospy.Publisher('/robot_status/console_msg', StringArray, queue_size=10)
        self._patrol_cancel_service = rospy.ServiceProxy('/patrol/cancel', Empty)

        self._current_status = GoalStatus.SUCCEEDED
        self._minimum_laser_range = -1.0
        self._current_retry = 1
        self._goal_header_id_during_incident = 0

        self._proximity_threshold = PROXIMITY_THRESHOLD
        self._dyn_rec_server = Server(SafetyWatchdogConfig, self._reconf_callback)

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

    def _reconf_callback(self, config, level):
        self._proximity_threshold = float(config.proximity_threshold)
        return config

    def _move_base_status_cb(self, msg: GoalStatusArray) -> None:
        if msg.status_list:
            self._current_status = msg.status_list[-1].status

    def _laser_scan_cb(self, msg: LaserScan) -> None:
        self._minimum_laser_range = min(msg.ranges)

    def _cancel_navigation(self) -> None:
        rospy.loginfo(f'Obstacle appeared in a distance of {self._minimum_laser_range}... Canceling navigation...')
        empty_goal = GoalID()
        for i in range(5):
            self._navigation_cancel_publisher.publish(empty_goal)

    def run(self) -> None:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._current_status == GoalStatus.ACTIVE and self._minimum_laser_range < self._proximity_threshold:
                self._patrol_cancel_service()
                self._cancel_navigation()
                console_msg = StringArray()
                console_msg.data.append(f'物体は{self._proximity_threshold}メートルの範囲に障害物を検知しました。ナビゲーションを終了します。')
                console_msg.data.append(f'周囲を確認し、新しい地点を指定して,Goボタンを押してください。')
                self._robot_status_msg_publisher.publish(console_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        sw = SafetyWatchdog()
        sw.run()
    except rospy.ROSInterruptException:
        pass
