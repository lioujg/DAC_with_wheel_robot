## Requirements
* Ubuntu 18.04 ros-melodic
* Gazebo 9 simulator

```
sudo apt-get install ros-melodic-joy
sudo apt-get install ros-melodic-octomap-ros
sudo apt-get install ros-melodic-mavlink
sudo apt-get install python-wstool
sudo apt-get install python-catkin-tools
sudo apt-get install protobuf-compiler
sudo apt-get install libgoogle-glog-dev
sudo apt-get install ros-melodic-control-toolbox
```

## Usage
* Create the world
```
roslaunch rotors_gazebo wheel_robots.launch
```
* Combine two robot with payload
```
rosrun gazebo_ros_link_attacher attach.py
```
* Enable the controller
```
roslaunch rotors_gazebo wheel_robots_controller.launch
```
