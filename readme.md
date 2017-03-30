# ROS Beginner Tutorials Project (Week10 HW)
[![Build Status](https://travis-ci.org/htsai51/beginner_tutorials.svg?branch=master)](https://travis-ci.org/htsai51/beginner_tutorials)
[![Coverage Status](https://coveralls.io/repos/github/htsai51/beginner_tutorials/badge.svg?branch=master)](https://coveralls.io/github/htsai51/beginner_tutorials?branch=master)
----

## Overview

This is a simple continuing ROS project which demonstrates the concepts of 

- Service
- Logging
- Launch Files

There are 2 nodes in this project.  Talker, the publisher node, registers to master to publish a message type of std_msgs/String on topic chatter.  Listener, the subscriber node, subscribes on chatter topic with master and prints out the message on screen.

## Dependencies

- Ubuntu 14.04
- ROS indigo
- Package Dependencies
    - roscpp
    - std_msgs

## Build

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ git clone --recursive https://github.com/htsai51/beginner_tutorials
$ cd ~/catkin_ws/
$ catkin_make
```

## Run

Open a new terminal to make sure roscore is running:

```bash
$ roscore
```
Open a new terminal to run talker:

```bash
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials talker
```
You will see something similar to:

```bash
viki@c3po:~/catkin_ws$ rosrun beginner_tutorials talker 
[ INFO] [1489805638.916941142]: [Jane] hello world 0
[ INFO] [1489805639.016930365]: [Jane] hello world 1
[ INFO] [1489805639.116847262]: [Jane] hello world 2
```

Open a new terminal to run listener:

```bash
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials listener
```

You will see something similar to:

```bash
viki@c3po:~/catkin_ws$ rosrun beginner_tutorials listener 
[ INFO] [1489805639.217746959]: [Jane] I heard: [[Jane] hello world 3]
[ INFO] [1489805639.317523213]: [Jane] I heard: [[Jane] hello world 4]
[ INFO] [1489805639.417634375]: [Jane] I heard: [[Jane] hello world 5]
```
