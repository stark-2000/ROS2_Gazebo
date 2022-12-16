[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---
## ros2_gazebo_TurtleBot3

## Overview:
 - This repository consists of Gazebo turtlebot 3 simulation.
 - Launch file is also created which launches all the nodes at once.
 - Google Style Guide was followed in all the cpp files.
 - CPPCHECK and CPPLINT was run to perform static code analysis.
 - "results" folder contains the output of CPPCHECK and CPPLINT in a text file.
 - doxygen documentation is added to the "docs" folder for all the cpp files.

## Dependencies/Requirements: 
 - Ubuntu 20.04 
 - VS Code/Terminal
 - ROS 2 Galactic
 - Gazebo 
 - Turtlesim

## Build & Run Instructions:
 - open terminal and source ros2
 ```
 source <ros2_galactic_install_directory> setup.bash
 ```
 - Create a workspace (if not created before)
 ```
 mkdir -p ~/ros2_ws/src
 cd ~/ros2_ws/src/
 ```
 - Navigate to root of workspace
 ```
 cd <ros2_workspace_name>
 ```
  - or
 ```
 cd ros2_ws
 ```
 - Check for missing dependencies before building the package
 ```
 rosdep install -i --from-path src --rosdistro galactic -y
 ```
 - Build the package
 ```
 colcon build --packages-select ros2_gazebo
 ```

 - open new terminal
 - Navigate to root of workspace
 ```
 cd ros2_ws
 ```
  - or
 ```
 cd <ros2_workspace_name>
 ```
 - source ros2
 ```
 . install/setup.bash
 ```
 - run launch file & record bag file
 ```
 ros2 launch ros2_gazebo launch.py record_flag:=True
 ```

## Command to run static code analysis:
 - Navigate to src folder in package
 ```
 cd ros2_ws/src/ros2_gazebo/src
 ```
 - run the following command from src folder in package
 ```
 cppcheck --enable=all --std=c++17 *.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ./../results/cppcheckreport
 ```

## Command to check Google Style:
 - Navigate to src folder in package
 ```
 cd ros2_ws/src/ros2_gazebo/src
 ```
 - run the following command from src folder in package
 ```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order Walker_Algo.cpp > ./../results/cpplintreport
 ```
 - Clang auto formating command:
 ```
 clang-format -style=Google -i gazeboWalker.cpp
 ```
## Dependency Installation: 
- ROS 2 Galactic:
- Follow the below website instructions to install ROS 2 Galactic based on your Ubuntu version
  - https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html

- Gazebo & Turtlesim:
```
sudo apt install ros-galactic-gazebo-ros-pkgs
sudo apt install ros-galactic-turtlebot3*
sudo apt install ros-galactic-gazebo-plugins
```

## Issues faced:
- Issues were mostly related to ROS2 Gazebo since we had initially installed ROS2 Humble which had issues with Gazebo andd Turtlebot. So we had to install ROS2 Galactic to complete this assignment.