#!/usr/bin/env python3

import os
import sys
import math
import rospy
import actionlib
from std_srvs.srv import Empty
from std_msgs.msg import Float32, Header
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PolygonStamped, Polygon, Point
import sensor_msgs.point_cloud2 as pc2
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from dynamic_reconfigure.server import Server
from telecoV.cfg import SafetyWatchdogConfig
from telecoV.msg import StringArray
from move_base_msgs.msg import MoveBaseAction

DEFAULT_ROBOT_RADIUS = 0.2
PROXIMITY_THRESHOLD = 0.5
VOLTAGE_THRESHOLD = 20.5


class SafetyWatchdog:
    def __init__(self) -> None:
        rospy.init_node("safety_watchdog")

        rospy.loginfo('Waiting for move_base')
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()

        self._current_status = GoalStatus.SUCCEEDED
        self._minimum_laser_range = -1.0
        self._current_retry = 1
        self._goal_header_id_during_incident = 0

        self._voltage_warning_flag = False
        self._proximity_threshold = PROXIMITY_THRESHOLD
        self._current_points_in_perimeter = list()
        self._dyn_rec_server = Server(SafetyWatchdogConfig, self._reconf_callback)

        self._robot_radius = rospy.get_param('/move_base/global_costmap/robot_radius', DEFAULT_ROBOT_RADIUS)
        rospy.Subscriber('/merged_cloud', PointCloud2, self._pc2_cb, queue_size=1)
        rospy.Subscriber('/voltage', Float32, self._voltage_cb, queue_size=1)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self._move_base_status_cb, queue_size=1)
        self._laser_scan_debug_pub = rospy.Publisher('/safety_watchdog/laser_scan_debug', PointCloud2, queue_size=1)
        self._safety_perimeter_poly_pub = rospy.Publisher('/safety_watchdog/safety_perimeter', PolygonStamped, queue_size=1, latch=True)
        self._robot_status_msg_publisher = rospy.Publisher('/robot_status/console_msg', StringArray, queue_size=10)
        self._patrol_cancel_service = rospy.ServiceProxy('/patrol/cancel', Empty)

        rospy.loginfo('Safety_Watchdog started')

    def _reconf_callback(self, config, level):
        self._proximity_threshold = float(config.proximity_threshold)
        return config

    def _publish_perimeter_poly(self) -> None:
        msg = PolygonStamped()
        msg.header = Header(stamp=rospy.Time.now(), frame_id='laser')
        msg.polygon = Polygon()
        msg.polygon.points.append(Point(x=0.0, y=self._robot_radius))
        msg.polygon.points.append(Point(x=self._proximity_threshold + self._robot_radius, y=self._robot_radius))
        msg.polygon.points.append(Point(x=self._proximity_threshold + self._robot_radius, y=-self._robot_radius))
        msg.polygon.points.append(Point(x=0.0, y=-self._robot_radius))
        self._safety_perimeter_poly_pub.publish(msg)

    def _move_base_status_cb(self, msg: GoalStatusArray) -> None:
        if msg.status_list:
            self._current_status = msg.status_list[-1].status

    def _voltage_cb(self, msg: Float32) -> None:
        if self._voltage_warning_flag:
            return

        if float(msg.data) < VOLTAGE_THRESHOLD:
            rospy.logwarn('Voltage approaching minimum. Consider end of operation.')
            self._voltage_warning_flag = True

    def _is_in_perimeter(self, point):
        if not -self._robot_radius <= point.y <= self._robot_radius:
            return False
        if not self._robot_radius < point.x <= self._proximity_threshold + self._robot_radius:
            return False
        return True

    def _pc2_cb(self, msg: PointCloud2) -> None:
        header = Header(stamp=rospy.Time.now(), frame_id='laser')
        point_list = pc2.read_points_list(msg, skip_nans=True, field_names=("x", "y", "z"))

        # Remove robot reflections
        point_list = [p for p in point_list if math.sqrt(p.x ** 2 + p.y ** 2) >= self._robot_radius]

        # Remove points outside of perimeter
        point_list = [p for p in point_list if self._is_in_perimeter(p)]

        point_fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1)]
        debug_msg = pc2.create_cloud(header, fields=point_fields, points=point_list)
        self._current_points_in_perimeter = point_list
        self._laser_scan_debug_pub.publish(debug_msg)

    def _warning_message(self):
        rospy.logwarn(f'Obstacle appeared in front of robot... Canceling navigation...')
        console_msg = StringArray()
        console_msg.data.append(f'物体は{self._proximity_threshold}メートルの範囲に障害物を検知しました。ナビゲーションを中断します。')
        console_msg.data.append(f'周囲を確認し、新しい地点を指定して,Goボタンを押してください。')
        self._robot_status_msg_publisher.publish(console_msg)

    def _cancel_navigation(self):
        self._patrol_cancel_service()
        self._move_base_client.cancel_all_goals()

    def _abort_conditions_met(self):
        return self._current_status == GoalStatus.ACTIVE and len(self._current_points_in_perimeter) > 0

    def run(self) -> None:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._publish_perimeter_poly()
            if self._abort_conditions_met():
                self._cancel_navigation()
                self._warning_message()
            rate.sleep()


if __name__ == '__main__':
    try:
        sw = SafetyWatchdog()
        sw.run()
    except rospy.ROSInterruptException:
        pass
