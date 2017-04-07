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

/**
 *  @file listener.cpp
 *  @brief File of main function for listener node
 *
 *  This file contains the main program of listener node which subscirbes
 *  to chatter topic.
 *
 *  This program demonstrate the concept of subscribing topic in ROS.
 *
 *
 *  @author Huei Tzu Tsai
 *  @date   03/30/2017
*/

#include "listener_class.hpp"
#include "beginner_tutorials/talkerService.h"


int main(int argc, char **argv) {
    bool ret;
    Listener listener;

    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    // Register client with the master
    ros::ServiceClient client =
        n.serviceClient<beginner_tutorials::talkerService>("talkerService");

    // Wait for service becomes ready (timeout 60 seconds)
    ret = ros::service::waitForService("talkerService", 5000);

    if (ret == false) {
        ROS_WARN_STREAM("Timout waiting for talkerService");
    } else {
        // Create request & response objects
        beginner_tutorials::talkerService::Request req;
        beginner_tutorials::talkerService::Response resp;

        // Fill in request data
        req.name = "Jane";

        // Call service
        ret = client.call(req, resp);

        // Check if service call is successful
        if (ret == true) {
            ROS_INFO_STREAM("Set talker name " << resp.resp);
        } else {
            ROS_ERROR_STREAM("Failed to set talker name");
        }
    }

    // Subscribe topic chatter from master to receive messages published
    // by this topic
    ros::Subscriber sub = n.subscribe("chatter", 1000,
                                      &Listener::chatterCallback, &listener);

    if (sub) {
        ROS_DEBUG_STREAM("Subscribe OK");
    } else {
        ROS_FATAL_STREAM("Subscribe FAILED.  Exit listener.");
        return -1;
    }

    ros::spin();

    return 0;
}

