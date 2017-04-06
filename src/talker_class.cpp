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

/** @file talker_class.cpp
 *  @brief Implementation of class Talker methods
 *
 *  This file contains implementation of talkerSerivce callback function
 *  in class Talker node.  talkerService updates the name in message
 *  publish by Talker node on chatter topic.
 *
 *  @author Huei Tzu Tsai
 *  @date   03/30/2017
*/

#include "talker_class.hpp"


bool Talker::updateTalkerName(
        beginner_tutorials::talkerService::Request &req,
        beginner_tutorials::talkerService::Response &resp) {
    name = req.name;
    resp.resp = "OK";
    ROS_INFO_STREAM("Update talker name to [" << name << "]");
    ROS_INFO_STREAM("Sending response " << resp.resp << " to client");
    return true;
}
