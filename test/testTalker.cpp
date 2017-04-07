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

/** @file test.cpp
 *  @brief Implementation of unit test for ROS node Talker
 *
 *  This file contains implementation of unit test for ROS node Talker
 *
 *  @author Huei Tzu Tsai
 *  @date   04/06/2017
*/

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "talker_class.hpp"
#include "beginner_tutorials/talkerService.h"


/**
 *   @brief  Verify talker service exists
 *
 *   @param  none
 *   @return none
*/
TEST(TestSuite, testTalkerServiceExist) {
    ros::NodeHandle n;

    // Register client with the master
    ros::ServiceClient client =
        n.serviceClient<beginner_tutorials::talkerService>("talkerService");

    // Assert service to be ready in 1 second
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1)));
}


/**
 *   @brief  Verify talker service accepts update to 
 *           name and response OK
 *
 *   @param  none
 *   @return none
*/
TEST(TestSuite, testTalkerServiceUpdate) {
    ros::NodeHandle n;

    // Register client with the master
    ros::ServiceClient client =
        n.serviceClient<beginner_tutorials::talkerService>("talkerService");

    // Create request & response objects
    beginner_tutorials::talkerService::Request req;
    beginner_tutorials::talkerService::Response resp;

    // Fill in request data
    req.name = "Jane";

    // Call service, expect return TRUE
    EXPECT_TRUE(client.call(req, resp));

    // Expect response is OK
    EXPECT_STREQ("OK", resp.resp.c_str());
}
