#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'core'
tmux new-window -t $SESSION:2 -n 'robot'
tmux new-window -t $SESSION:3 -n 'cameras'
tmux new-window -t $SESSION:4 -n 'ui'
tmux new-window -t $SESSION:5 -n 'navigation'
tmux new-window -t $SESSION:6 -n 'scheduler'
tmux new-window -t $SESSION:7 -n 'scheduler_service'
tmux new-window -t $SESSION:8 -n 'semantic_map'
tmux new-window -t $SESSION:9 -n 'exploration'


tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "DISPLAY=:0 roslaunch mongodb_store mongodb_store.launch db_path:=/opt/strands/aaf_datacentre"

tmux select-window -t $SESSION:2
tmux send-keys "DISPLAY=:0 roslaunch strands_bringup strands_robot.launch with_mux:=false"

tmux select-window -t $SESSION:3
tmux send-keys "DISPLAY=:0 roslaunch strands_bringup strands_cameras.launch head_camera:=true head_ip:=left-cortex head_user:=strands chest_camera:=true chest_ip:=right-cortex chest_user:=strands"

tmux select-window -t $SESSION:4
tmux send-keys "rosparam set /deployment_language english && HOST_IP=192.168.0.100 DISPLAY=:0 roslaunch strands_ui strands_ui.launch mary_machine:=right-cortex mary_machine_user:=strands"

tmux select-window -t $SESSION:5
tmux send-keys "DISPLAY=:0 roslaunch icra_start aaf_navigation.launch map:=/localhome/strands/icra16data/iros/iros_base.yaml topological_map:=WW_GF_2015_09_08"

tmux select-window -t $SESSION:6
tmux send-keys "DISPLAY=:0 roslaunch task_executor task-scheduler-mdp.launch"

tmux select-window -t $SESSION:7
tmux send-keys "DISPLAY=:0 rosservice call /task_executor/set_execution_status"

tmux select-window -t $SESSION:8
tmux send-keys "DISPLAY=:0 ssh left-cortex" C-m
tmux send-keys "DISPLAY=:0 roslaunch semantic_map_launcher semantic_map.launch"

tmux select-window -t $SESSION:9
tmux send-keys "DISPLAY=:0 ssh left-cortex" C-m
tmux send-keys "DISPLAY=:0 rosrun spatiotemporal_exploration spatiotemporal_exploration"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
