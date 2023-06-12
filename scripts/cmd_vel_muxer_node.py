#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3


class CmdVelMuxer:
    def __init__(self) -> None:
        rospy.init_node("cmd_vel_muxer_node")
        rospy.Subscriber('/rover_twist_cylinder', Twist, self._cylinder_twist_cb, queue_size=10)
        rospy.Subscriber('/rover_twist_base', Twist, self._base_twist_cb, queue_size=10)
        self._twist_publisher = rospy.Publisher('/rover_twist', Twist, queue_size=10)
        rospy.loginfo('Robot_Status started')
        self._combined_twist_message = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self._twist_received = False

    def _cylinder_twist_cb(self, msg: Twist) -> None:
        self._combined_twist_message.linear.z = msg.linear.z
        self._twist_received = True

    def _base_twist_cb(self, msg: Twist) -> None:
        self._combined_twist_message.linear.x = msg.linear.x
        self._combined_twist_message.linear.y = msg.linear.y
        self._combined_twist_message.angular.z = msg.angular.z
        self._twist_received = True

    def run(self) -> None:
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._twist_received:
                self._twist_publisher.publish(self._combined_twist_message)
            rate.sleep()


if __name__ == '__main__':
    try:
        cvm = CmdVelMuxer()
        cvm.run()
    except rospy.ROSInterruptException:
        pass
