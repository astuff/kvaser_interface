/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <ros/ros.h>
#include <kvaser_interface/kvaser_interface.h>
#include <can_msgs/Frame.h>

can_msgs::Frame test_frame_1,
  test_frame_2,
  test_frame_3,
  test_frame_4;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Kvaser Interface Testing Node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Populate test frames
  test_frame_1.header.seq = 0;
  test_frame_1.id = 0x555;
  test_frame_1.dlc = 8;

  for (auto i = 0; i < test_frame_1.dlc; ++i)
  {
    test_frame_1.data[i] = (i % 2 == 0) ? 1 : 0;
  }

  test_frame_2.header.seq = 1;
  test_frame_2.id = 0x555;
  test_frame_2.dlc = 4;

  for (auto i = 0; i < test_frame_1.dlc; ++i)
  {
    test_frame_1.data[i] = (i % 2 == 0) ? 1 : 0;
  }

  return 0;
} 