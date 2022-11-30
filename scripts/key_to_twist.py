#! /usr/bin/env python3

"""keysを購読し，twistを配信する"""

import rospy
# import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys
from select import select
import termios
import tty

#キーの割当：[angular.z, linear.x]
key_mapping = {'w':[ 0, 1], 'x':[0, -1],
               'a':[-1, 0], 'd':[1,  0],
               's':[ 0, 0]}

def getKey(settings, timeout):
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select([sys.stdin], [], [], timeout)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def saveTerminalSettings():
	return termios.tcgetattr(sys.stdin)  # termios はターミナルI/O用の Unix API
	# シリアルポートの開閉
	# 通信のパラメータの設定
	# シリアル通信による読み書き

def restoreTerminalSettings(old_settings):
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    settings = saveTerminalSettings()
    rospy.init_node('keys_to_twist')    #ノードの初期化

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = TwistStamped

    twist_pub = rospy.Publisher('rover_twist', Twist, queue_size = 10)   #cmd_vel配信準備

    try:
        while(1):
        	key = getKey(settings, key_timeout)
        	print('➡  ' + key + ' was pused --- \n')
        	if key in key_mapping.keys():
        		x = key_mapping[key][0]
        		ang_z = key_mapping[key][1]
        		# z = key_mapping[key][2]

        	else:
        		if (key == '\x03'):
        			break
			# rospy.Subscriber('keys', String, keys_callback, twist_pub)  #keysを購読し，コールバック関数を呼び出す．引数はさらに後ろで指定する
			# rospy.spin()    #ループcmd_vel

    except Exception as e:
        print(e)

    finally:
        #pub_thread.stop()
        restoreTerminalSettings(settings)
