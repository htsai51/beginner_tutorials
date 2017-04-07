# ROS Beginner Tutorials Project
[![Build Status](https://travis-ci.org/htsai51/beginner_tutorials.svg?branch=Week11_HW)](https://travis-ci.org/htsai51/beginner_tutorials)
[![Coverage Status](https://coveralls.io/repos/github/htsai51/beginner_tutorials/badge.svg?branch=master)](https://coveralls.io/github/htsai51/beginner_tutorials?branch=master)
----

## Overview

This is a simple ROS project which demonstrates the concept of 

- Nodes
- Topics
- Publisher/Subscriber
- Service
- Logging
- Launch Files
- TF
- Rostest
- Rosbag

There are 2 nodes in this project.

[Talker]
- The publisher node, registers to master to publish a message type of std_msgs/String on topic chatter.
- Talker provides a service called "talkerService" to update name published in chatter topic.
- Talker broadcasts a TF frame /talker with reference to /world frame

[Listener]
- The subscriber node, subscribes on chatter topic with master and prints out the message on screen.
- Listener implements a client to call "talkerService" to set the name published in chatter topic before it subscribes to chatter topic.

## Dependencies

- Ubuntu 14.04
- ROS indigo
- Package Dependencies
    - roscpp
    - std_msgs
    - message_generation
    - tf
    - rostest

## Build

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ git clone --recursive https://github.com/htsai51/beginner_tutorials -b Week11_HW
$ cd ~/catkin_ws/
$ catkin_make
```

## Run - rosrun

Use rosrun to run talker & listener nodes as well as set publish freq

Open a new terminal to make sure roscore is running:

```bash
$ roscore
```
Open a new terminal to run talker:

```bash
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials talker 1
```
You will see something similar to:

```bash
viki@c3po:~/catkin_ws$ rosrun beginner_tutorials talker 1
[ INFO] [1490937615.332981621]: freq configured to 1Hz
```

Open a new terminal to run listener:

```bash
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials listener
```

You will see something similar to:

```bash
[ INFO] [1490937753.744361100]: Set talker name OK
[ INFO] [1490937754.742746190]: I heard: [Jane] hello world 4
[ INFO] [1490937755.742402745]: I heard: [Jane] hello world 5
[ INFO] [1490937756.742433061]: I heard: [Jane] hello world 6
```

## Run - roslaunch

Use roslaunch to run talker and listener nodes together and set publish freq on chatter topic

Open a new terminal to make sure roscore is running:

```bash
$ roscore
```
Open a new terminal to run launch file:

```bash
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ roslaunch beginner_tutorials tutorial.launch freq:=1
```

You will see 2 output terminals similar below:

Talker terminal

```bash
viki@c3po:~/catkin_ws$ roslaunch beginner_tutorials tutorial.launch freq:=1
... logging to /home/viki/.ros/log/d0268856-159a-11e7-bf24-080027b46cdd/roslaunch-c3po-20981.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://c3po:47343/

SUMMARY
========

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.8

NODES
  /
    listener1 (beginner_tutorials/listener)
    talker1 (beginner_tutorials/talker)

ROS_MASTER_URI=http://localhost:11311

core service [/rosout] found
process[talker1-1]: started with pid [20999]
[ INFO] [1490937154.376445743]: freq configured to 1Hz
process[listener1-2]: started with pid [21011]
[listener1-2] process has finished cleanly
log file: /home/viki/.ros/log/d0268856-159a-11e7-bf24-080027b46cdd/listener1-2*.log
[ INFO] [1490937155.391733645]: Update talker name to [Jane]
[ INFO] [1490937155.391803726]: Sending response OK to client

```

Listener terminal

```bash
[ INFO] [1490937208.149806024]: Set talker name OK
[ INFO] [1490937209.149474912]: I heard: [Jane] hello world 2
[ INFO] [1490937210.150257105]: I heard: [Jane] hello world 3
[ INFO] [1490937211.149289031]: I heard: [Jane] hello world 4
```

## Run - Call Service

Make sure talker & listener nodes are already running

Open a new terminal to check if talkerService is available

```bash
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosservice list
```

You should see output similiar to below.

```bash
viki@c3po:~/catkin_ws$ rosservice list
/listener/get_loggers
/listener/set_logger_level
/rosout/get_loggers
/rosout/set_logger_level
/rqt_gui_py_node_19460/get_loggers
/rqt_gui_py_node_19460/set_logger_level
/talker/get_loggers
/talker/set_logger_level
/talkerService
```

Call talkerService to update name on chatter topic

```bash
rosservice call /talkerService George
```

On Talker node terminal, the output should look like:

```bash
[ INFO] [1490938098.932018948]: Update talker name to [George]
[ INFO] [1490938098.932098616]: Sending response OK to client
```

On Listener node terminal, the name should be updated and the output should look like:

```bash
[ INFO] [1490938097.932338667]: I heard: [Jane] hello world 42
[ INFO] [1490938098.932710332]: I heard: [Jane] hello world 43
[ INFO] [1490938099.932610406]: I heard: [George] hello world 44
[ INFO] [1490938100.932359965]: I heard: [George] hello world 45
```

## TF

Talker node broadcasts a TF frame /talker with reference to /world frame.  To inspect TF frame published by /talker:

In a new terminal
```bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials talker
```

Open another terminal

```bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun tf tf_echo /world /talker
```

The output should look like
```bash
At time 1491533084.395
- Translation: [1.732, 1.001, 0.000]
- Rotation: in Quaternion [1.000, 0.000, 0.000, 0.000]
            in RPY (radian) [3.142, -0.000, 0.000]
            in RPY (degree) [180.000, -0.000, 0.000]
At time 1491533085.394
- Translation: [1.779, -0.915, 0.000]
- Rotation: in Quaternion [1.000, 0.000, 0.000, 0.000]
            in RPY (radian) [3.142, -0.000, 0.000]
            in RPY (degree) [180.000, -0.000, 0.000]
```

TF tree can be viewed using

rqt_tf_tree

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

OR view_frames

```bash
rosrun tf view_frames
evince frames.pdf
```

## Test - rostest

Level 2 integration test implemented by gtest can be run on talker service as follow:

To build and run the test:

```bash
cd ~/catkin_ws
catkin_make tests
source ./devel/setup.bash
rostest beginner_tutorials testTalker.launch
```

OR

```bash
cd ~/catkin_ws
source ./devel/setup.bash
catkin_make run_tests_beginner_tutorials
```

## Play/Record - rosbag

rosbag can be used to record and play topic messages.

rosbag is included in tutorial.launch to record all topics but is disabled by default.

To enable rosbag recording:

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch beginner_tutorials tutorial.launch enable_record:=true freq:=1
```

To inspect rosbag recording result:

```bash
cd ~/.ros/
rosbag info session.bag
```

The output should look like
```bash
path:        session.bag
version:     2.0
duration:    19.8s
start:       Apr 06 2017 23:37:55.31 (1491536275.31)
end:         Apr 06 2017 23:38:15.08 (1491536295.08)
size:        44.1 KB
messages:    128
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      20 msgs    : std_msgs/String   
             /rosout       46 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   42 msgs    : rosgraph_msgs/Log 
             /tf           20 msgs    : tf2_msgs/TFMessage
```

To playback rosbag recording:

In a new terminal
```bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```

Open another terminal
```bash
cd ~/.ros/
rosbag play session.bag
```

Listner terminal should output messages recorded by rosbag as below:

```bash
viki@c3po:~/catkin_ws$ rosrun beginner_tutorials listener
[ INFO] [1491538729.909485813]: waitForService: Service [/talkerService] has not been advertised, waiting...
[ WARN] [1491538734.926901138]: Timout waiting for talkerService
[ INFO] [1491538739.068361724]: I heard: [] hello world 1
[ INFO] [1491538740.067775213]: I heard: [Jane] hello world 2
[ INFO] [1491538741.069624190]: I heard: [Jane] hello world 3
[ INFO] [1491538742.071395132]: I heard: [Jane] hello world 4
```