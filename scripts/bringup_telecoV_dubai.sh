#!/bin/bash
session="telecoV"
base_window_name="bring_up"
capf_window_name="capf"
use_waypoint_server_cli="true"
tmux new-session -d -s $session -n $base_window_name;
tmux split-window -v
tmux split-window -h
tmux split-window -v
tmux split-window -v
tmux send-keys -t $session:$base_window_name.0 "roslaunch telecoV navigation_teb_diff.launch rover_type:=mega3 start_rviz:=false map_file:=dubai_dfa.yaml" Enter
sleep 5
tmux send-keys -t $session:$base_window_name.1 "roslaunch telecoV waypoint_server.launch cli:=$use_waypoint_server_cli" Enter
tmux send-keys -t $session:$base_window_name.2 "roslaunch telecoV patrol.launch" Enter
tmux send-keys -t $session:$base_window_name.3 "roslaunch telecoV robot_status.launch" Enter
tmux send-keys -t $session:$base_window_name.4 "roslaunch telecoV convenience_server.launch" Enter
tmux new-window -n $capf_window_name;
tmux split-window -v
tmux split-window -v
tmux split-window -h
tmux send-keys -t $session:$capf_window_name.0 "roslaunch takasaki_capf teleco_basic.launch stress_test:=false" Enter
# tmux send-keys -t $session:$capf_window_name.1 "rosrun telecoV safety_watchdog.py" Enter
tmux send-keys -t $session:$capf_window_name.1 "rviz -d ~/catkin_ws/src/teleco_ros/rviz/sim.rviz" Enter
tmux send-keys -t $session:$capf_window_name.2 "roscd takasaki_capf/scripts ; rosrun takasaki_capf lipsync.py" Enter
tmux send-keys -t $session:$capf_window_name.3 "roscd takasaki_capf ; rosrun takasaki_capf capf_auto_login.py" Enter
tmux attach -t "$session:$base_window_name"
