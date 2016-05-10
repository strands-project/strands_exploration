#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongodb'
tmux new-window -t $SESSION:2 -n 'morse'
tmux new-window -t $SESSION:3 -n '2d_nav'
tmux new-window -t $SESSION:4 -n 'cameras'
tmux new-window -t $SESSION:5 -n 'navigation'
tmux new-window -t $SESSION:6 -n 'strands_ui'
tmux new-window -t $SESSION:7 -n ''


tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "source ~/tsc_ws/devel/setup.bash; DISPLAY=:0 roslaunch mongodb_store mongodb_store.launch db_path:=$HOME/mongodb_store"

tmux select-window -t $SESSION:2
tmux send-keys "source ~/tsc_ws/devel/setup.bash; DISPLAY=:0 roslaunch strands_morse uol_ww_morse.launch"

tmux select-window -t $SESSION:3
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch strands_morse uol_ww_nav2d.launch"

tmux select-window -t $SESSION:4
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch strands_morse generate_camera_topics.launch"

tmux select-window -t $SESSION:5
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch strands_ui strands_ui.launch"

tmux select-window -t $SESSION:6
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch backtrack_behaviour backtrack.launch"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
