# ROS2 HDR

In this project, a holonomic drive robot called duke is designed and simulated to learn ROS2 and Nav2 Stacks.

In this project, we have designed the duke bot for exploring the ros2 navigation stack. Feel free to use this repo for your project.

## Pre-requisite

- [ROS2 Humble 🔗](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) with Gazebo

    While installing ROS2 make sure you use this command.

    ```
    sudo apt install ros-humble-desktop-full # Instead sudo apt install ros-humble-desktop
    ```

    Check whether Gazebo-11 is installed using the following command.
    ```
    gazebo --version
    ```

If the gazebo is not recognized, install gazebo-11 using this [link](https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros). Make sure to use the appropriate distro in the command.

- ROS2 binary packages

    ```
    sudo apt-get install ros-humble-rosidl-typesupport-c
    sudo apt-get install ros-humble-joint-state-publisher
    sudo apt-get install ros2-controllers -y
    sudo apt-get install ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations
    sudo pip3 install transforms3d
    ```

## Sourcing

source ROS2 and Gazebo `setup.bash` in the `~/.bashrc` file.

    # For Colcon Auto-Completion
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    # ROS 2 Bash
    source /opt/ros/humble/setup.bash
    # Gazebo 11 Bash
    source /usr/share/gazebo-11/setup.sh
    # ROS2 Workspace
    source ~/<ROS2-Workspace>/install/setup.bash

## Repository Structure

* assets – images for docs are added.
* designs – duke bot design in Fusion360, SolidWorks and SketchUp Format are attached.
* duke_bot_description – ROS2 python package containing all the files related to holonomic drive robot for simulation
* hdr_navigation – ROS2 python package to explore the navigation stack using the duke bot in gazebo simulation
* nav_world – ROS2 CPP package containing all the files related to add simulation world in the gazebo

## Demonstration

<img src="./assets/demo.gif"/>

### Project Members

Hari Vikinesh, Jerish Abijith Singh
