#!/usr/bin/env python3

import os
import sys
import math
import numpy as np
import cv2
import rospy
import actionlib
from actionlib_msgs.msg import GoalID, GoalStatusArray, GoalStatus
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose2D, Pose, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetMap
from sensor_msgs.msg import Image
from telecoV.msg import RobotStatus, WaypointArray
from telecoV.srv import (GoalHeadingService, GoalHeadingServiceResponse, RelativeTurnService, RelativeTurnServiceResponse,
                         GoalHeadingByLabelService, GoalHeadingByLabelServiceResponse,
                         MapVisualizationService, MapVisualizationServiceResponse)
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError

REFERENCE_FRAME = 'map'
ROBOT_FRAME = 'base_link'


class ConvenienceServer:
    def __init__(self) -> None:
        rospy.init_node("convenience_server_node")

        self._latest_robot_status = RobotStatus()
        self._latest_robot_status_received = False
        self._latest_waypoints = dict()
        self._latest_waypoints_received = False

        self._reference_frame = rospy.get_param('/convenience_server/reference_frame', REFERENCE_FRAME)
        self._robot_frame = rospy.get_param('/convenience_server/robot_frame', ROBOT_FRAME)

        rospy.loginfo('Waiting for move_base')
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()

        rospy.loginfo('Waiting for static_map')
        rospy.wait_for_service('static_map')
        self._world_map_client = rospy.ServiceProxy('static_map', GetMap)

        self._goal_heading_debug_publisher = rospy.Publisher('/convenience_server/debug/goal_heading', PoseStamped, queue_size=10)
        self._debug_map_pub = rospy.Publisher('/convenience_server/debug_image_map', Image, queue_size=2, latch=True)
        self._cv_bridge = CvBridge()

        rospy.Subscriber('/robot_status/status', RobotStatus, self._robot_status_cb, queue_size=1)
        rospy.Subscriber('/waypoint_server/waypoints', WaypointArray, self._waypoint_cb, queue_size=1)
        self._goal_heading_service = rospy.Service('/convenience_server/goal_heading', GoalHeadingService, self._goal_heading_service_cb)
        self._goal_heading_by_label_service = rospy.Service('/convenience_server/goal_heading_by_label', GoalHeadingByLabelService, self._goal_heading_by_label_service_cb)
        self._relative_turn_service = rospy.Service('/convenience_server/relative_turn', RelativeTurnService, self._relative_turn_service_cb)
        self._map_visualization_service = rospy.Service('/convenience_server/map_visualization', MapVisualizationService, self._map_visualization_service_cb)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        rospy.loginfo('Convenience_Server started')

    def _map_visualization_service_cb(self, msg: MapVisualizationService) -> MapVisualizationServiceResponse:
        latest_map = self._world_map_client()
        image = np.array(latest_map.map.data).reshape(latest_map.map.info.height, latest_map.map.info.width).astype(dtype=np.uint8)
        image = cv2.flip(image, 0)
        color_mapping = {0: 255, 100: 0, 255: 130}
        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                image[i][j] = color_mapping[image[i][j]]
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        response: MapVisualizationServiceResponse = MapVisualizationServiceResponse(success=True)
        origin_px: (int, int) = (int(latest_map.map.info.origin.position.x / latest_map.map.info.resolution) * -1,
                                 latest_map.map.info.height + int(latest_map.map.info.origin.position.y / latest_map.map.info.resolution))

        if msg.draw_map_origin:
            self._draw_arrow(image, origin_px, (0, 0), 0.0, (0, 255, 0))

        if self._latest_waypoints_received:
            for itm in self._latest_waypoints.items():
                position_px_offset = self._coord_to_px_offset(image, origin_px, latest_map.map.info.resolution, (itm[1].pose.position.x,
                                                                                                                 itm[1].pose.position.y))
                _, _, yaw = euler_from_quaternion([itm[1].pose.orientation.x, itm[1].pose.orientation.y,
                                                   itm[1].pose.orientation.z, itm[1].pose.orientation.w])
                response.labels.append(itm[0])
                response.pixel_positions.append(Pose2D(x=origin_px[0] + position_px_offset[0],
                                                       y=origin_px[1] - position_px_offset[1], theta=yaw))
                if msg.draw_waypoint_positions:
                    self._draw_arrow(image, origin_px, position_px_offset, yaw, (255, 0, 255))

        if msg.draw_robot_position and self._latest_robot_status_received:
            latest_pose = self._latest_robot_status.current_position
            position_px_offset = self._coord_to_px_offset(image, origin_px, latest_map.map.info.resolution, (latest_pose.pose.position.x,
                                                                                                             latest_pose.pose.position.y))
            _, _, yaw = euler_from_quaternion([latest_pose.pose.orientation.x, latest_pose.pose.orientation.y,
                                               latest_pose.pose.orientation.z, latest_pose.pose.orientation.w])
            self._draw_arrow(image, origin_px, position_px_offset, yaw, (0, 0, 255))

        img_msg = self._cv_bridge.cv2_to_imgmsg(image, 'rgb8')
        self._debug_map_pub.publish(img_msg)
        response.visualization = img_msg
        return response

    def _draw_arrow(self, image, origin_px: (int, int), position: (int, int), rotation: float, color: (int, int, int),
                    arrow_length: int = 20, thickness: int = 2, tip_length: float = 0.4) -> None:

        start_point = (origin_px[0] + position[0], origin_px[1] - position[1])
        rot_off = (int(math.cos(rotation) * arrow_length),int(math.sin(rotation) * arrow_length))
        end_point: (int, int) = (start_point[0] + rot_off[0], start_point[1] - rot_off[1])
        cv2.arrowedLine(image, start_point, end_point, color, thickness, tipLength=tip_length)

    def _coord_to_px_offset(self, image, origin_px: (int, int), resolution: float, coord: (float, float)) -> (int, int):
        x = int((coord[0] / resolution))
        y = int((coord[1] / resolution))

        x = -origin_px[0] if origin_px[0] + x < 0 else x
        x = image.shape[1] - 1 - origin_px[0] if origin_px[0] + x >= image.shape[1] else x
        y = origin_px[1] if origin_px[1] - y < 0 else y
        y = -(image.shape[0] - 1 - origin_px[1]) if origin_px[1] + -y >= image.shape[0] else y

        return x, y

    def _robot_status_cb(self, msg: RobotStatus) -> None:
        self._latest_robot_status = msg
        self._latest_robot_status_received = True

    def _waypoint_cb(self, msg: WaypointArray) -> None:
        latest_waypoints = dict()
        for w in msg.waypoints:
            latest_waypoints[w.label] = w.pose
        self._latest_waypoints = latest_waypoints
        self._latest_waypoints_received = True

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
        return GoalHeadingServiceResponse(self._get_goal_heading(msg.goal_pose))

    def _goal_heading_by_label_service_cb(self, msg: GoalHeadingByLabelService) -> GoalHeadingByLabelServiceResponse:
        goal_heading_quaternion = Quaternion(0, 0, 0, 0)
        success = False
        try:
            goal_pose = self._latest_waypoints[msg.goal_label]
            goal_heading_quaternion = self._get_goal_heading(goal_pose)
            success = True
        except KeyError:
            pass

        return GoalHeadingByLabelServiceResponse(goal_heading=goal_heading_quaternion, success=success)

    def _get_goal_heading(self, goal_pose: PoseStamped) -> Quaternion:
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

        if self._goal_heading_debug_publisher.get_num_connections() > 0:
            debug_pose = PoseStamped()
            debug_pose.header.stamp = rospy.Time.now()
            debug_pose.header.frame_id = self._robot_frame
            debug_pose.pose.position = Point()
            debug_pose.pose.orientation = goal_heading_quaternion
            self._goal_heading_debug_publisher.publish(debug_pose)

        return goal_heading_quaternion

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
