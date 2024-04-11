# REQUIRED INSTALLATIONS BEFORE USE:

1. ROS2 Humble
2. Moveit2 for ROS Humble
   - Command for BINARY INSTALL (recommended):
      -   sudo apt install ros-humble-moveit
3. ROS2 Packages for Robot Simulation and Control
   - ROS2 Control + ROS2 Controllers:
      -   sudo apt install ros-humble-ros2-control
      -   sudo apt install ros-humble-ros2-controllers
      -   sudo apt install ros-humble-gripper-controllers   
    
   - Gazebo for ROS2 Humble:
      -   sudo apt install gazebo
      -   sudo apt install ros-humble-gazebo-ros2-control
      -   sudo apt install ros-humble-gazebo-ros-pkgs
    
   - xacro:
      -   sudo apt install ros-humble-xacro
    
   - Fix cycle time issues in humble-moveit (temporary fix):
      -   sudo apt install ros-humble-rmw-cyclonedds-cpp # (+) Add into .bashrc file -> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
