#!/bin/zsh
sleep 1;
source /opt/ros/noetic/setup.zsh

sleep 1;
sudo chmod 777 /dev/ttyACM0

sleep 1;

# BYHUAV 开启
gnome-terminal --tab --title="GINS" -- zsh -c "ulimit -c unlimited && echo core-%e-%p-%t | sudo tee /proc/sys/kernel/core_pattern && source ~/Ros/gins_ws/devel/setup.zsh && roslaunch byh_uav byh_uav.launch; exec zsh" && 
sleep 1 &&
# 输出信息 
gnome-terminal --tab --title="ICM42688" -- zsh -c "source ~/Ros/gins_ws/devel/setup.zsh && rostopic echo /byh_uav/ICM42688 ; exec zsh" &&
gnome-terminal --tab --title="RM3100" -- zsh -c "source ~/Ros/gins_ws/devel/setup.zsh && rostopic echo /byh_uav/RM3100 ; exec zsh" && 
gnome-terminal --tab --title="BMP581" -- zsh -c "source ~/Ros/gins_ws/devel/setup.zsh && rostopic echo /byh_uav/BMP581 ; exec zsh" &&
gnome-terminal --tab --title="ZEDF9P" -- zsh -c "source ~/Ros/gins_ws/devel/setup.zsh && rostopic echo /byh_uav/ZEDF9P ; exec zsh" &&

echo "base_serial  successfully started"

sleep 1;
wait;
