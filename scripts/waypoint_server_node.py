#!/usr/bin/env python3

import os
import sys
import copy
import pickle
from typing import Dict
import rospy
from telecoV.msg import Waypoint, WaypointArray
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

DEFAULT_PATH = '/home/arne/waypoints.bin'
initial_position_name = 'init'


class WaypointMarkerHandler:
    def __init__(self, change_callback=None):
        self._server = InteractiveMarkerServer("waypoint_marker_handler")
        self._menu_handler = MenuHandler()
        self._menu_handler.insert("Navigate Here", callback=self._process_feedback)
        self._menu_handler.insert("Remove", callback=self._process_feedback)

        self._marker_changed_callback = change_callback
        self._server.applyChanges()

    def _process_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        self._marker_changed_callback(feedback)
        self._server.applyChanges()

    def _make_marker(self, msg: InteractiveMarker) -> Marker:
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale.x = msg.scale * 0.5
        marker.scale.y = msg.scale * 0.1
        marker.scale.z = msg.scale * 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def _make_box_control(self, msg: InteractiveMarker) -> InteractiveMarkerControl:
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self._make_marker(msg))
        msg.controls.append(control)
        return control

    def _normalize_quaternion(self, quaternion_msg: Quaternion) -> None:
        norm = quaternion_msg.x ** 2 + quaternion_msg.y ** 2 + quaternion_msg.z ** 2 + quaternion_msg.w ** 2
        s = norm ** (-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

    def add_goal_pose_marker(self, initial_pose: PoseStamped, label: str) -> None:
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose = initial_pose.pose
        int_marker.scale = 1

        int_marker.name = label
        int_marker.description = label

        self._make_box_control(int_marker)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        self._normalize_quaternion(control.orientation)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        self._normalize_quaternion(control.orientation)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        self._normalize_quaternion(control.orientation)
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        # BUG: Mixing InteractiveMarkerControl.MOVE_PLANE with InteractiveMarkerControl.ROTATE_AXIS controls screws up marker interaction
        # control = InteractiveMarkerControl()
        # control.orientation.w = 1
        # control.orientation.x = 0
        # control.orientation.y = 1
        # control.orientation.z = 0
        # normalizeQuaternion(control.orientation)
        # control.name = "move_plane"
        # control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        # int_marker.controls.append(copy.deepcopy(control))

        self._server.insert(int_marker, self._process_feedback)
        self._menu_handler.apply(self._server, int_marker.name)
        self._server.applyChanges()

    def remove_goal_marker(self, label: str) -> None:
        self._server.erase(label)


class WaypointServer:
    def __init__(self) -> None:
        rospy.init_node("waypoint_server_node")
        rospy.on_shutdown(self._shutdown_hook)
        rospy.loginfo('Waypoint_Server started')
        self._waypoints = self._load_waypoints(DEFAULT_PATH)

        self._navigation_cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self._navigation_goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

        self._shortcut_mapping = {'add': (self._add_waypoint, 'Add waypoint at 0,0,0 with name of argument', 1),
                                  'remove': (self._remove_waypoint, 'Removes waypoint with name of argument', 1),
                                  'list': (self._list_waypoints, 'Lists all waypoints', 0),
                                  'stop': (self._stop_navigation, 'Stops current navigation', 0),
                                  'cls': (self._clear_screen, 'Clear screen', 0),
                                  'quit': (self._end_program, 'Quit', 0)
                                  }

        self._mh = WaypointMarkerHandler(change_callback=self._marker_changed_cb)
        for w in self._waypoints:
            waypoint = self._waypoints[w]
            self._mh.add_goal_pose_marker(waypoint.pose, str(waypoint.label))

        self._running = True

    def _shutdown_hook(self) -> None:
        self._save_waypoints(DEFAULT_PATH, self._waypoints)

    def _marker_changed_cb(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                self._send_navigation_goal(feedback.marker_name)
            elif feedback.menu_entry_id == 2:
                self._remove_waypoint(feedback.marker_name)
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._update_waypoint(feedback.marker_name, feedback.pose)

    def _load_waypoints(self, path: str) -> Dict[str, Waypoint]:
        try:
            with open(path, 'rb') as infile:
                ret = pickle.load(infile)
                return ret
        except FileNotFoundError:
            return {initial_position_name: Waypoint('init', PoseStamped())}

    def _save_waypoints(self, path: str, waypoints: Dict[str, Waypoint]) -> None:
        with open(path, 'wb') as outfile:
            pickle.dump(waypoints, outfile)

    def _add_waypoint(self, name: str) -> None:
        if name in self._waypoints.keys():
            print(f'Waypoint "{name}" aleady exists')
            return
        wp = Waypoint(name, PoseStamped())
        wp.pose.header.frame_id = 'map'
        self._waypoints[name] = wp
        self._mh.add_goal_pose_marker(wp.pose, wp.label)

    def _remove_waypoint(self, name: str) -> None:
        try:
            self._waypoints.pop(name)
            self._mh.remove_goal_marker(name)
        except KeyError:
            print(f'Waypoint "{name}" doesn\'t exist')

    def _update_waypoint(self, label, pose):
        if label in self._waypoints:
            self._waypoints[label].pose.pose = pose

    def _stop_navigation(self) -> None:
        print('Stopping navigation')
        empty_goal = GoalID()
        for i in range(5):
            self._navigation_cancel_publisher.publish(empty_goal)

    def _list_waypoints(self):
        for w in self._waypoints.keys():
            print(f'{w} -> [x:{self._waypoints[w].pose.pose.position.x}, y:{self._waypoints[w].pose.pose.position.y}, z:{self._waypoints[w].pose.pose.position.z}]')

    def _end_program(self) -> None:
        self._shutdown_hook()
        print('Exiting.....')
        self._clear_screen()
        self._running = False

    def _clear_screen(self) -> None:
        os.system('clear')

    def _send_navigation_goal(self, waypoint_label) -> None:
        move_base_goal = MoveBaseActionGoal()
        move_base_goal.header = Header()
        move_base_goal.goal.target_pose = self._waypoints[waypoint_label].pose
        move_base_goal.goal.target_pose.header.frame_id = 'map'
        self._navigation_goal_publisher.publish(move_base_goal)

    def _help(self) -> None:
        print('Usage: command [argument]')
        for k in self._shortcut_mapping:
            print(f'{k}\t-> {self._shortcut_mapping[k][1]}')

    def run(self) -> None:
        while self._running:
            print('>>> ', end='')
            val = input()
            try:
                args = val.split(' ')
                mapping = self._shortcut_mapping[args[0]]
                if mapping[2] == 0:
                    mapping[0]()
                elif mapping[2] == 1:
                    mapping[0](args[1])
            except (KeyError, AttributeError, IndexError):
                self._help()


if __name__ == '__main__':
    try:
        ws = WaypointServer()
        ws.run()
    except rospy.ROSInterruptException:
        pass
