/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <ros/ros.h>
#include <kvaser_interface/kvaser_interface.h>
#include <can_msgs/Frame.h>
#include <gtest/gtest.h>

#include <vector>

std::vector<can_msgs::Frame::ConstPtr> rcvd_msgs;

TEST(ROSKvaserInterface, MessageContents)
{
  ASSERT_EQ(rcvd_msgs.size(), 3) << "Incorrect number of valid messages received.";

  for (const auto& msg : rcvd_msgs)
  {
    switch (msg->header.seq)
    {
      case 0:
      {
        ASSERT_EQ(msg->id, 0x555) << "Got incorrect message ID on seq 0.";
        ASSERT_EQ(msg->dlc, 8) << "Got incorrect DLC on seq 0.";
        ASSERT_FALSE(msg->is_rtr) << "Got unexptected is_rtr value on seq 0.";
        ASSERT_FALSE(msg->is_extended) << "Got unexpected is_extended value on seq 0.";
        ASSERT_FALSE(msg->is_error) << "Got unexpected is_error value on seq 0.";

        for (auto i = 0; i < msg->data.size(); ++i)
        {
          if (i % 2 == 0)
            ASSERT_EQ(msg->data[i], 0x55) << "Got unexpected payload value on seq 0, byte " << i;
          else
            ASSERT_EQ(msg->data[i], 0xAA) << "Got unexpected payload value on seq 0, byte " << i;
        }
      }
      break;
      case 1:
      {
        ASSERT_EQ(msg->id, 0x555) << "Got incorrect message ID on seq 1.";
        ASSERT_EQ(msg->dlc, 8) << "Got incorrect DLC on seq 1.";
        ASSERT_FALSE(msg->is_rtr) << "Got unexptected is_rtr value on seq 1.";
        ASSERT_FALSE(msg->is_extended) << "Got unexpected is_extended value on seq 1.";
        ASSERT_FALSE(msg->is_error) << "Got unexpected is_error value on seq 1.";

        for (auto i = 0; i < msg->data.size(); ++i)
        {
          ASSERT_EQ(msg->data[i], 0) << "Got unexpected payload value on seq 1, byte " << i;
        }
      }
      break;
      case 2:
        FAIL() << "Received message seq 2 which should have caused a write failure.";
        break;
      case 3:
      {
        ASSERT_EQ(msg->id, 0x15555555) << "Got incorrect message ID on seq 3.";
        ASSERT_EQ(msg->dlc, 8) << "Got incorrect DLC on seq 3.";
        ASSERT_FALSE(msg->is_rtr) << "Got unexptected is_rtr value on seq 3.";
        ASSERT_TRUE(msg->is_extended) << "Got unexpected is_extended value on seq 3.";
        ASSERT_FALSE(msg->is_error) << "Got unexpected is_error value on seq 3.";

        for (auto i = 0; i < msg->data.size(); ++i)
        {
          if (i % 2 == 0)
            ASSERT_EQ(msg->data[i], 0x55) << "Got unexpected payload value on seq 3, byte " << i;
          else
            ASSERT_EQ(msg->data[i], 0xAA) << "Got unexpected payload value on seq 3, byte " << i;
        }
      }
      break;
      case 4:
        FAIL() << "Received message seq 4 which should have caused a write failure.";
        break;
      default:
        FAIL() << "Received message seq " << msg->header.seq << " which was unexpected.";
    }
  }
}

void reader_callback(const can_msgs::Frame::ConstPtr& msg)
{
  std::cout << "********* GOT A MESSAGE **********" << std::endl;
  rcvd_msgs.push_back(msg);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "Kvaser Interface Testing Node");
  ros::NodeHandle nh;

  auto msg_pub = nh.advertise<can_msgs::Frame>("/writer3/can_rx", 20);
  auto msg_sub = nh.subscribe("/reader3/can_tx", 20, reader_callback);

  // Wait until reader and writer are ready
  while (true)
  {
    if (msg_pub.getNumSubscribers() < 1 &&
        msg_sub.getNumPublishers() < 1)
      ros::Duration(0.1).sleep();
    else
      break;
  }

  can_msgs::Frame can_msg;

  // Standard ID and DLC matches payload
  can_msg.header.seq = 0;
  can_msg.id = 0x555;
  can_msg.dlc = 8;

  for (auto i = 0; i < can_msg.dlc; ++i)
  {
    can_msg.data[i] = (i % 2 == 0) ? 0x55 : 0xAA;
  }

  msg_pub.publish(can_msg);

  // Standard ID and DLC does not match payload
  can_msg.header.seq = 1;
  can_msg.dlc = 0;

  msg_pub.publish(can_msg);

  // Standard ID but is_extended = true
  // Should not be received (write failure)
  can_msg.header.seq = 2;
  can_msg.is_extended = true;
  can_msg.dlc = 8;

  msg_pub.publish(can_msg);

  // Extended ID and is_extended = true
  can_msg.header.seq = 3;
  can_msg.id = 0x15555555;

  msg_pub.publish(can_msg);

  // Extended ID and is_extended = false
  // Should not be received (write failure)
  can_msg.header.seq = 4;
  can_msg.is_extended = false;

  msg_pub.publish(can_msg);

  auto later = ros::Time::now() + ros::Duration(3.0);

  // Spin for 3 seconds, then run tests
  while (later > ros::Time::now())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return RUN_ALL_TESTS();
}
