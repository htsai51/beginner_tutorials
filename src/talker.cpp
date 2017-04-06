/********************************************************************
 *   MIT License
 *  
 *   Copyright (c) 2017 Huei-Tzu Tsai
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 ********************************************************************/

#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>
#include "talker_class.hpp"
#include "beginner_tutorials/talkerService.h"


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
    // set default publish freq to 10 Hz
    int freq = 10;

    ros::init(argc, argv, "talker");

    // Check launch argument
    if (argc > 1) {
        ROS_DEBUG_STREAM("argv[1] is " << argv[1]);
        freq = atoi(argv[1]);
    }

    ROS_INFO_STREAM("freq configured to " << freq << "Hz");

    ros::NodeHandle n;
    Talker talker;

    // Register service with the master
    ros::ServiceServer server =
        n.advertiseService("talkerService", &Talker::updateTalkerName, &talker);


    // Register to master to publish topic chatter
    // Anyone subscribing to this topic name will receive messages
    // published by this topic
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(freq);

    // A count of how many messages we have sent
    int count = 0;

    while (ros::ok()) {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "[" << talker.getName() << "] hello world " << count;
        msg.data = ss.str();

        ROS_DEBUG_STREAM("Publish: " << msg.data.c_str());

        // send messages
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}

