#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point

import sys
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QApplication, QWidget
# sudo apt install python3-pyqt5

POS_Z = 0.0
isUpdate = False

def pose_callback(msg_pose):
    global POS_Z
    POS_Z = msg_pose.z
    # print("[current height]" + str(POS_Z))

def value_s():
    label.setText(str(slider.value()))

def update():
    isUpdate = True
    value = slider.value()
    print("[*] Update Height！" + str(value))

    ## 04_入力を得て目標値を設定する
    goal_h = value
    # goal_h = float(input('Set goal height: betweeb 2 ~ 300  > ')) # 2 ~ 300 キー入力から取得

    ## 03_現在地点と目標地点を比較してどっちに行くか決める
    rospy.wait_for_message("cylinder_pos", Point, timeout=None)

    # goal_h = 2   # 2 ~ 300 でお願いします
    current_h = POS_Z
    print("[current height] " + str(POS_Z) + " / [goal height] " + str(goal_h))

    vel_h = Twist()
    if current_h < goal_h :
        vel_h.linear.z = 0.1
        print("🙇‍♀️上へ参ります。")
    
    if current_h > goal_h :
        vel_h.linear.z = -0.1
        print("🙇‍♂️下へ参ります。")

    while not rospy.is_shutdown():
        ## 02_目標地点を与えてそこまで動かす
        if (goal_h-10 < POS_Z <= goal_h):
            vel_h.linear.z = 0
            pub.publish(vel_h)
            isUpdate = False
            print("[END height] " + str(POS_Z))
            break

        ## 01_上下に 最大/最小 までとにかく動かす
        # vel_h.linear.z = 0.1
        pub.publish(vel_h)
        rate.sleep()


if __name__ == '__main__':
    ## GUI設定
    app = QApplication(sys.argv)
    root = QWidget()
    root.setWindowTitle('シリンダー上げ下げ')

    slider = QtWidgets.QSlider(root)
    slider.setGeometry(30,50,200,300)
    slider.setValue(100)
    slider.setMinimum(2)
    slider.setMaximum(300)
    slider.valueChanged.connect(value_s)

    update_button = QtWidgets.QPushButton(root)
    update_button.setText("Update")
    update_button.clicked.connect(update)
    update_button.move(150,150)

    label_style = """QLabel {
            color: #FFFFFF;                 /* 文字色 */
            font-size: 32px;               /* 文字サイズ */
            background-color:#2196F3;
            border-color:#FF9800;
            border-radius:4px;
            align-center;
        }"""
    label = QtWidgets.QLabel(root)
    label.setStyleSheet(label_style)
    label.resize(72, 32)
    label.setAlignment(QtCore.Qt.AlignCenter)
    label.move(154,100)

    ## ROS設定
    rospy.init_node('cylinder')
    pub = rospy.Publisher('rover_twist', Twist, queue_size=10)
    pub_pos = rospy.Publisher('cylinder_now', Point, queue_size=10)
    sub = rospy.Subscriber("cylinder_pos", Point, pose_callback)
    rate = rospy.Rate(30)

    root.show()
    sys.exit(app.exec_())