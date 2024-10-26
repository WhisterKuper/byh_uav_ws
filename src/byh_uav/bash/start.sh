#!/bin/zsh
sleep 1;

gnome-terminal -- zsh -c "zsh /home/kuper/Ros/byh_uav_ws/src/byh_uav/bash/start_script.sh; exec zsh" 

sleep 1;

wait;
