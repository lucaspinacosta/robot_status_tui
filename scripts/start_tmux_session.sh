#!/bin/bash
tmux new-session -d -s robot_status
tmux split-window -v
tmux send-keys -t 0 "roslaunch robot_status_tui robot_status.launch" C-m
tmux attach-session -t robot_status
