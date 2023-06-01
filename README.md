# このパッケージの用途
created by ©︎niscode

ここではTeleco-V（台車）を使って、
- gmappingを用いたSLAM や、
- dwa（障害物回避）と自己位置推定（amcl）を用いたナビゲーション などを実行するためのlaunchファイルが格納されます。
<br>

## [1/3] Ubuntu20.04 / ROS-noetic　環境を構築するための手順
- `git clone git@github.com:niscode/scripts.git`
- 上記のリポジトリをホームディレクトリに配置後、$ ./ros-noetic-setup.sh を実行。
- 手順通りに進めると　$ roscore できるようになります。
<br>

## [2/3] Ubuntu20.04 / ROS-noetic　環境で動作に必要なパッケージ群は以下の通りです。
- `sudo apt -y install ros-noetic-rosserial`
- `sudo apt -y install ros-noetic-slam-gmapping`
- `sudo apt -y install ros-noetic-navigation`
- `ros-noetic-map-server`
- `ros-noetic-jsk-visualization`

<br>

- `cd catkin_ws/src`
- `git clone https://github.com/niscode/telecoV.git`
- `git clone https://github.com/Slamtec/rplidar_ros.git`
- `git clone https://github.com/iralabdisco/ira_laser_tools.git`
- `git clone https://github.com/GT-RAIL/robot_pose_publisher.git`
<br>

- `cd catkin_ws`
- `catkin_make`
<br>

## [3/3] USB接続されたデバイスファイル名の固定するための手順 （melodic/noetic共通）
- `roscd telecoV/scripts`
- `nano rplidar.rules`
- 上記ファイルを編集して、対応するRPLiDARや台車のボードのシリアルNoを指定します。
- `./create_udev_rules.sh`
- `sudo reboot`
<br>

### gmappingによる地図作成は以下のように実行します。
- `roslaunch telecoV dual_gmapping.launch`
### navigationによる自律移動は以下のように実行します。
- `roslaunch telecoV dual_naivation.launch`

### cylinder昇降用スライダを表示するには以下を実行します。
- `rosrun telecoV cylinder.py`
<br>![シリンダー上下用のスライダー](img/slider.png)
- スライダーを上下に動かし、任意の位置で Update ボタンを押すことで動作します。
- ※ `rosrun rosserial_python serial_node.py _port:=/dev/ROVER_BOARD _baud:=115200` が実行中か、
    navigationなどのパッケージ実行中のみ、昇降が可能です。

## Additional Node Descriptions
### _waypoint_server_node.py_
This node offers local waypoint setting, serializing, de-serializing and editing functions (CRUD). It publishes the most recent set of waypoints on the topic “/waypoint_server/waypoints” and uses InteractiveMarkers to allow easy editing of the waypoints with rviz. It offers its full functionality via an optional local CLI (check help command) and exposes basic functions through service calls.

Uses: __waypoint_server.launch__

#### Subscribed Topics
clicked_point (geometry_msgs::PointStamped)

&emsp;Adds point with automatic name when using rviz’s “Publish Point” function.

#### Published Topics
waypoint_server/waypoints (telecoV.msg::WaypointArray)

&emsp;Publishes most recent local waypoints

move_base/goal (move_base_msgs::MoveBaseActionGoal)

&emsp;Used for “Navigate Here” function of InteractiveMarkers

move_base/cancel (actionlib_msgs::GoalID) 

&emsp;Used for the “stop” navigation function of the CLI

#### Services
waypoint_server/add (telecoV.srv::WaypointService)

&emsp;Adds a new waypoint at 0,0,0 with name of argument string.

waypoint_server/remove (telecoV.srv::WaypointService)

&emsp;Removes waypoint with name of argument string.

waypoint_server/add_here (telecoV.srv::WaypointService)

&emsp;Adds a new waypoint robot position with name of argument string.

#### Parameters
waypoint_server/waypoint_filepath (string, default: "/tmp/waypoints.bin")

&emsp;Filepath of were to safe the waypoints after node gets closed.

waypoint_server/reference_frame (string, default: "map")

&emsp;Reference frame for the waypoints.

waypoint_server/robot_frame (string, default: "base_link")

&emsp;Robot frame the transform from the reference frame is used of.

waypoint_server/cli (bool, default: "False")

&emsp;Whether to spawn a command line in the console. If it is used it should be ended via the "quit" command before Ctrl-C kills the node.

### _patrol_node.py_
This node offers functionality to control patrolling over a given waypoint list.

Uses: __patrol.launch__

#### Subscribed Topics
waypoint_server/waypoints (telecoV.msg::WaypointArray)

&emsp;Used to always patrol to the most up-to-date waypoints.

move_base/status (actionlib_msgs::GoalStatusArray)

&emsp;Internally used to determine behavior based on navigation state.

#### Services
patrol/start (telecoV.srv::PatrolService)

&emsp;If empty -> random patrol, if one waypoint -> drive to point and end patrol, if len(waypoints) > 1 -> drive to waypoints in given order and start over at end.

patrol/cancel (std_srvs::Empty)

&emsp;Ends patrol after reaching current waypoint.

#### Parameters
waypoint_server/reference_frame (string, default: "map")

&emsp;Reference frame for the waypoints.

waypoint_server/robot_frame (string, default: "base_link")

&emsp;Robot frame the transform from the reference frame is used of.

### _safety_watchdog_simple.py_
This node constantly monitors the LaserScan data published on “/scan” and if proximity falls short of threshold cancels the patrol and the current navigation so that the robot stops. The teleoperator has to make sure  the robot gets moved out of the obstacle and then can restart the patrol.

#### Subscribed Topics
scan (sensor_msgs::LaserScan)

&emsp;Subscribes to laserScan topic and checks for proximity violations.

#### Published Topics
move_base/cancel (actionlib_msgs::GoalID)

&emsp;In case of obstacle is detected closer than the proximity threshold cancel current navigation and stop robot at its current location.

#### Services
patrol/cancel (std_srvs::Empty)

&emsp;In case of obstacle is detected closer than the proximity threshold calls to end current patrol.

### _robot_status_node.py_
This node collects information from several node and re-sends them in a custom message as a central information point.

Uses: __robot_status.launch__

#### Subscribed Topics
move_base/status (move_base_msgs::GoalStatusArray)

&emsp;Listens to get the latest status of the navigation stack.

move_base/goal (move_base_msgs::MoveBaseActionGoal)

&emsp;Listens to know the latest navigation goal.

robot_status/console_msg (telecoV.msg::StringArray)

&emsp;Listens to custom string array message which can be appended to a log window on the operator side.

#### Published Topics
robot_status/status (telecoV.msg::RobotStatus)

&emsp;Please check message definition for content information.

#### Parameters
waypoint_server/reference_frame (string, default: "map")

&emsp;Reference frame for the waypoints.

waypoint_server/robot_frame (string, default: "base_link")

&emsp;Robot frame the transform from the reference frame is used of.

### _conveniences_server_node.py_
This node offers basic functions for the base, which can be used to support the interaction.

Uses: __convenience_server.launch__

#### Subscribed Topics
robot_status/status (telecoV.msg::RobotStatus)

&emsp;For future use.

#### Published Topics
convenience_server/debug/goal_heading (geometry_msgs::PoseStamped)

&emsp;Publishes the requested heading for debug purposes.

#### Services
convenience_server/goal_heading (telecoV.srv::GoalHeadingService)

&emsp;Requests the relative rotation of the robot in reference to the given goal.

convenience_server/relative_turn (telecoV.srv::RelativeTurnService)

&emsp;Uses move_base to turn the robot in-place by the given rotation value.

#### Parameters
waypoint_server/reference_frame (string, default: "map")

&emsp;Reference frame for the waypoints.

waypoint_server/robot_frame (string, default: "base_link")

&emsp;Robot frame the transform from the reference frame is used of.