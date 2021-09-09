#!/bin/sh
xterm  -e  " roslaunch my_home_service_robot my_robot_world.launch " &
sleep 5
xterm  -e  " roslaunch my_home_service_robot amcl_demo.launch " & 
sleep 5
xterm  -e  " roslaunch rvizConfig view_navigation.launch " &
sleep 5
xterm  -e  " roslaunch pick_objects pick_objects.launch " 
