# BasketBot
My project for the Udacity Robotics Software Engineer Nanodegree 2019.

## Getting Started
You need [ROS](http://ros.org). If you aren't familiar with [ROS](http://ros.org) then this is probably no good to you whatsoever!

I have used ROS Melodic but it has been tested on ROS Kinetic.

I'm not going to talk you through how to use ROS, there are plenty of tutorials online and the [ROS Wiki](http://wiki.ros.org) is particularly useful). You will need a ROS workspace, either one you have currently or [create a new workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

You will also need [RVIZ](http://wiki.ros.org/rviz/UserGuide) and [Gazebo](http://gazebosim.org/).


### Prerequisites
- An installed ROS system with rviz and gazebo
- A ROS workspace

### Installing
You should [create a new package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage).

`catkin_create_pkg my_robot roscpp std_msgs message_generation`

Copy this project into the my_robot directory.

## Usage
You will need 2 terminals running.

**Terminal 1**
In your ROS workspace :-

```
source devel/setup.bash
catkin_make
roslaunch my_robot world.launch
```

**Terminal 2**
In your ROS workspace :-

```
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```

`ball_chaser.launch` can take 3 parameters, the RGB values of an object for the robot to chase. e.g.

```
roslaunch ball_chaser ball_chaser.launch red:=102 green:=0 blue:=0
```

## Contributing
No contributions accepted this is **my project** for the course.

## License
GNU GPLv3

## Attributions & Acknowlegements
