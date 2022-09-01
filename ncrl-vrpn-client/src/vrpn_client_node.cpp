/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2015, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "vrpn_client_ros/vrpn_client_ros.h"
#include "vrpn_client_ros/serial.hpp"
#include <string.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vrpn_client_node");
  ros::NodeHandle nh, private_nh("~");
  vrpn_client_ros::VrpnClientRos client(nh, private_nh);
#if 0
  string port_s;
  if(nh.getParam("port", port_s) == false) {
    ROS_FATAL("No serial port is assigned.");
    exit(0);
  }

  string baudrate_s;
  if(nh.getParam("baudrate", baudrate_s) == false) {
    ROS_FATAL("No serial baudrate is assigned.");
    exit(0);
  }

  int baudrate = stoi(baudrate_s);
  serial_init((char *)port_s.c_str(), baudrate);
#endif
  reg_serial_with_marker(1, (char *)"/dev/ttyUSB0", 115200);
  //reg_serial_with_marker(2, (char *)"/dev/ttyUSB1", 115200);
 
 ros::spin();
  return 0;
}

