#!/bin/bash
session="telecoV"
window_name="bring_up"
use_waypoint_server_cli="true"
tmux new-session -d -s $session -n $window_name;
tmux split-window -v
tmux split-window -h
tmux split-window -v
tmux split-window -v
tmux send-keys -t 0 "roslaunch telecoV navigation_teb_diff.launch start_rviz:=false" Enter
sleep 5
tmux send-keys -t 1 "roslaunch telecoV waypoint_server.launch cli:=$use_waypoint_server_cli" Enter
tmux send-keys -t 2 "roslaunch telecoV patrol.launch" Enter
tmux send-keys -t 3 "roslaunch telecoV robot_status.launch" Enter
tmux send-keys -t 4 "roslaunch telecoV convenience_server.launch" Enter
tmux attach -t "$session"
