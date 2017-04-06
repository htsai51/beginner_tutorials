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

/** @file talker_class.hpp
 *  @brief Definition of class Talker
 *
 *  This file contains definitions of class Talker which is a node that
 *  registers to master to publish chatter topic
 *
 *  @author Huei Tzu Tsai
 *  @date   03/30/2017
*/

#ifndef INCLUDE_TALKER_CLASS_HPP_
#define INCLUDE_TALKER_CLASS_HPP_

#include <stdlib.h>
#include <ros/ros.h>
#include <string>
#include "beginner_tutorials/talkerService.h"


/**
 *  @brief Class definition of Talker node
*/
class Talker {
 public:
     /**
      *   @brief  Callback function of talkerService which updates
      *           the name in message published by Talker node on
      *           chatter topic
      *
      *   @param  name string to be udpated in message
                  published on chatter topic
      *   @param  response string to client
      *   @return true if update is successful, false otherwise
     */
     bool updateTalkerName(
         beginner_tutorials::talkerService::Request &,
         beginner_tutorials::talkerService::Response &);

     /**
      *   @brief  Get name to include in message published by Talker
      *           node
      *
      *   @param  none
      *   @return name string to be published on chatter topic
     */
     std::string getName(void) { return name; }

 private:
     std::string name;    ///< name in message published on chatter topic
};

#endif  // INCLUDE_TALKER_CLASS_HPP_
