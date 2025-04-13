#!/bin/zsh
sleep 1;
source /opt/ros/noetic/setup.zsh

sleep 1;
sudo ifconfig eth0 192.168.1.50
sudo ifconfig eth1 192.168.0.144
sudo chmod 777 /dev/ttyTHS0
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyCH9344USB8
sudo chmod 777 /dev/ttyCH9344USB9
sudo chmod 777 /dev/ttyCH9344USB10
sudo chmod 777 /dev/ttyCH9344USB11
sudo chmod 777 /dev/ttyCH9344USB12
sudo chmod 777 /dev/ttyCH9344USB13
sudo chmod 777 /dev/ttyCH9344USB14
sudo chmod 777 /dev/ttyCH9344USB15

sleep 1;

# 开启ptp主时钟同步
gnome-terminal --tab --title="ptp同步" -- zsh -c "sudo ptp4l -m -S -l 6 -i eth0; exec zsh" && 
# BYHUAV 开启
gnome-terminal --tab --title="BYHUAV" -- zsh -c "ulimit -c unlimited && echo core-%e-%p-%t | sudo tee /proc/sys/kernel/core_pattern && source ~/Ros/byh_uav_ws/devel/setup.zsh && roslaunch byh_uav byh_uav2.launch; exec zsh" && 
sleep 1 &&
# PPS同步 开启
gnome-terminal --tab --title="PPS同步" -- zsh -c "sudo -u root zsh -c \" ulimit -c unlimited && echo core-%e-%p-%t | sudo tee /proc/sys/kernel/core_pattern && source /home/kuper/Ros/byh_uav_ws/devel/setup.zsh  && rosrun byh_uav_pps2 byh_uav_pps2 /dev/pps1 \"; exec zsh" && 
sleep 1 &&
# 相机触发 开启
gnome-terminal --tab --title="Trigger" -- zsh -c "ulimit -c unlimited && echo core-%e-%p-%t | sudo tee /proc/sys/kernel/core_pattern && source ~/Ros/byh_uav_ws/devel/setup.zsh && roslaunch byh_uav byh_uav_trigger.launch; exec zsh" && 
# MID360 开启
# gnome-terminal --tab --title="MID360" -- zsh -c "ulimit -c unlimited && echo core-%e-%p-%t | sudo tee /proc/sys/kernel/core_pattern && sudo ifconfig eth0 192.168.1.50 && roslaunch livox_ros_driver2 rviz_MID360.launch; exec zsh" && 
# MID70 开启
# gnome-terminal --tab --title="MID70" -- zsh -c "ulimit -c unlimited && echo core-%e-%p-%t | sudo tee /proc/sys/kernel/core_pattern && sudo ifconfig eth0 192.168.1.50 && roslaunch livox_ros_driver livox_lidar_rviz.launch; exec zsh" && 
# ARS548 开启
# gnome-terminal --tab --title="ARS548" -- zsh -c "sudo -u root zsh -c \" ulimit -c unlimited && echo core-%e-%p-%t | sudo tee /proc/sys/kernel/core_pattern && sudo ifconfig eth0 192.168.1.50 && source /home/kuper/Ros/ars548_ws/devel/setup.zsh && roslaunch ars540_msgs ars540.launch  \"; exec zsh" && 
# FLIR 开启
# gnome-terminal --tab --title="FLIR" -- zsh -c "roslaunch spinnaker_sdk_camera_driver acquisition.launch; exec zsh" && 
# 输出信息 
gnome-terminal --tab --title="BMI088" -- zsh -c "source ~/Ros/byh_uav_ws/devel/setup.zsh && rostopic echo /byh_uav/BMI088 ; exec zsh" &&
gnome-terminal --tab --title="AK8975" -- zsh -c "source ~/Ros/byh_uav_ws/devel/setup.zsh && rostopic echo /byh_uav/AK8975 ; exec zsh" &&
gnome-terminal --tab --title="BMP581" -- zsh -c "source ~/Ros/byh_uav_ws/devel/setup.zsh && rostopic echo /byh_uav/BMP581 ; exec zsh" &&
gnome-terminal --tab --title="ADIS16470" -- zsh -c "source ~/Ros/byh_uav_ws/devel/setup.zsh && rostopic echo /byh_uav/ADIS16470 ; exec zsh" && 
gnome-terminal --tab --title="RM3100" -- zsh -c "source ~/Ros/byh_uav_ws/devel/setup.zsh && rostopic echo /byh_uav/RM3100 ; exec zsh" && 
gnome-terminal --tab --title="ICM42688" -- zsh -c "source ~/Ros/byh_uav_ws/devel/setup.zsh && rostopic echo /byh_uav/ICM42688 ; exec zsh" &&
# gnome-terminal --tab --title="SPL06" -- zsh -c "source ~/Ros/byh_uav_ws/devel/setup.zsh && rostopic echo /byh_uav/SPL06 ; exec zsh" &&
gnome-terminal --tab --title="ZEDF9P" -- zsh -c "source ~/Ros/byh_uav_ws/devel/setup.zsh && rostopic echo /byh_uav/ZEDF9P ; exec zsh" &&
gnome-terminal --tab --title="FPGA" -- zsh -c "source ~/Ros/byh_uav_ws/devel/setup.zsh && rostopic echo /byh_uav/FPGA ; exec zsh"  &&
gnome-terminal --tab --title="PPS_ALL" -- zsh -c "source ~/Ros/byh_uav_ws/devel/setup.zsh && rostopic echo /byh_uav/pps_sys ; exec zsh"  

echo "ptp sync successfully started"
echo "base_serial  successfully started"
echo "MID360  successfully started"
echo "MID70  successfully started"
echo "ARS548  successfully started"
echo "FLIR  successfully started"

sleep 1;
wait;
