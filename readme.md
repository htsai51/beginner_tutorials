# ROS Beginner Tutorials Project (Week10 HW)
[![Build Status](https://travis-ci.org/htsai51/beginner_tutorials.svg?branch=Week10_HW)](https://travis-ci.org/htsai51/beginner_tutorials)
[![Coverage Status](https://coveralls.io/repos/github/htsai51/beginner_tutorials/badge.svg?branch=master)](https://coveralls.io/github/htsai51/beginner_tutorials?branch=master)
----

## Overview

This is a simple ROS project which demonstrates the concept of 

- Service
- Logging
- Launch Files

There are 2 nodes in this project.

[Talker]
The publisher node, registers to master to publish a message type of std_msgs/String on topic chatter.  In addition, Talker provides a service called "talkerService" 
to update name published in chatter topic.

[Listener]
The subscriber node, subscribes on chatter topic with master and prints out the message on screen.  Listener also implements a client to call "talkerService" to 
set the name published in chatter topic before it subscribes to chatter topic.

## Dependencies

- Ubuntu 14.04
- ROS indigo
- Package Dependencies
    - roscpp
    - std_msgs
    - message_generation

## Build

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ git clone --recursive https://github.com/htsai51/beginner_tutorials -b Week10_HW
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