# RoboND-HomeServiceRobot-Project
Udacity Udacity Robotics Software Engineer Nanodegree Final Project

This project implements a home service robot scenario for autonomous navigation of a skid-steer mobile robot 
to pickup and drop off zones of a virtual object. 

## Installation

Install xterm 

    sudo apt-get install xterm 
    
Clone and compile project files to/in your "catkin_ws"/src folder

    git clone git clone https://github.com/AlmasShintemirov/RoboND-HomeServiceRobot-Project.git 

    cd ..

    catkin_make

    source devel/setup.bash

## Running

Shell scripts located in src/scripts folder are used for executing corresponding ROS nodes (launch files in packages) in 
separate xterm terminal windows. 

The robot uses ROS Gmapping (http://wiki.ros.org/gmapping) and Teleop_twist_keyboard (http://wiki.ros.org/teleop_twist_keyboard) packages for autonomous mapping of a custom Gazebo world environment ( test_slam.sh shell script ).

Adaptive Monte Carlo Localisation (ROS AMCL package http://wiki.ros.org/amcl) is used for robot localization and 
deploying ROS Navigation stack for navigate to prediefined object pick up and drop off zones. 
Motion planning is done using GlobalPlanner Teb local planner (http://wiki.ros.org/teb_local_planner). 
Use test_navigation.sh shell script ( my_service_robot package ) for testing the robot navigation to a manually 
commanded 2D NAV Goal arrow in Rviz with a configuration settings stored in rvizConfig package.

The pick_objects node ( pick_objects package ) communicates with the ROS navigation stack and autonomously sends
successive object pickup and drop off goals for the robot to reach. The node displays a message when the robot 
has reached its pickup destination, waits 5 seconds, then travels to the desired drop off zone, and displays 
another message that it has reached the drop off zone. Use pick_objects.sh shell script for testing the scenario. 

The add_markers node ( add_markers package ) models a virtual object with a cube marker in Rviz 
The virtual object marker first appears in its pickup zone and then in its drop off zone once the robot reaches it.
Use add_markers.sh shell script to run the node and marker visualization in Rviz. 

The final scenario is deployed using home_service.sh shell script. 
It follows the following algorithm:

Initially show the marker at the pickup zone
Hide the marker once your robot reaches the pickup zone
Wait 5 seconds to simulate a pickup
Show the marker at the drop off zone once your robot reaches it

![Screenshot from 2021-09-07 20-48-12](https://user-images.githubusercontent.com/13367696/132625138-75cd826b-93b8-47d8-941d-755a8768e534.png)
![Screenshot from 2021-09-07 20-49-16](https://user-images.githubusercontent.com/13367696/132625157-5f2ed563-be5d-43f8-8f68-57a8639a69d5.png)
![Screenshot from 2021-09-07 20-49-45](https://user-images.githubusercontent.com/13367696/132625161-9e3f2a8c-54d2-4a46-9f82-951b528b45aa.png)


