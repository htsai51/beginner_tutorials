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

/** @file listener_class.hpp
 *  @brief Definition of class Listener
 *
 *  This file contains definition of class Listener which is a node that
 *  subscribes to chatter topic and print messages received
 *  from chatter
 *
 *  @author Huei Tzu Tsai
 *  @date   03/30/2017
*/

#ifndef INCLUDE_LISTENER_CLASS_HPP_
#define INCLUDE_LISTENER_CLASS_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>


/**
 *  @brief Class definition of Listener node
*/
class Listener {
 public:
     /**
      *   @brief  Callback function of Listener class which
      *           subscibe to chatter topic
      *
      *   @param  message string received from chatter topic
      *   @return none
     */
     void chatterCallback(const std_msgs::String::ConstPtr&);
};

#endif  // INCLUDE_LISTENER_CLASS_HPP_
