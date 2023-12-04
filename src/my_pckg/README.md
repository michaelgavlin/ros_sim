# readme

# Blue White homework

## 1. Installation

### 1.1 Install TurtleBot3

- Install dependent ROS packages

```jsx
$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
ros-noetic-rosserial-python ros-noetic-rosserial-client \
ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

- Install TurtleBot3 packages

```jsx
$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3
```

- `source ~/.bashrc`

For full instructions if something fails:

https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

### 1.2 clone this package and build

- Clone this repo to`~/catkin_ws/`
- `cd ~/catkin_ws/src/my_pckg`
- `catkin_make`

### 1.3 To use my customized robot (Optional)

To use my customized robot copy the following files, adjust the env paths based on your setup.

You can use the out of the box robot without the special appearance.

Copy xacro and files to your ros directory:

- `sudo cp meshes/turtlebot3_burger_michael.urdf.xacro /opt/ros/noetic/share/turtlebot3_description/urdf`
- `sudo cp meshes/turtlebot3_burger_michael.gazebo.xacro /opt/ros/noetic/share/turtlebot3_description/urdf`

Copy launch file 

- `cp meshes/turtlebot3_empty_world_michael.launch ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch`

## 2. Launch

- Gazebo enviroment with my robot:

`roslaunch turtlebot3_gazebo turtlebot3_empty_world_michael.launch`

- Gazebo enviroment with basic robot:

`roslaunch turtlebot3_gazebo turtlebot3_empty_world_michael.launch`

- Launch all nodes in `my_pckg`

Note: user_interface_node is optional if using the provided GUI and is mainly for debgging.