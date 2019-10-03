// Copyright (c) 2019 AutonomouStuff, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <kvaser_interface/kvaser_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/Frame.hpp>

#include <gtest/gtest.h>

#include <vector>

std::vector<can_msgs::Frame::ConstPtr> rcvd_msgs;

TEST(ROSKvaserInterface, MessageContents)
{
  ASSERT_EQ(rcvd_msgs.size(), 4) << "Incorrect number of valid messages received.";

  // Hoping we receive them in the same order they were sent.
  // MSG 1
  ASSERT_EQ(rcvd_msgs[0]->id, 0x555) << "Got incorrect message ID on msg 1.";
  ASSERT_EQ(rcvd_msgs[0]->dlc, 8) << "Got incorrect DLC on msg 1.";
  ASSERT_FALSE(rcvd_msgs[0]->is_rtr) << "Got unexptected is_rtr value on msg 1.";
  ASSERT_FALSE(rcvd_msgs[0]->is_extended) << "Got unexpected is_extended value on msg 1.";
  ASSERT_FALSE(rcvd_msgs[0]->is_error) << "Got unexpected is_error value on msg 1.";

  for (auto i = 0; i < rcvd_msgs[0]->data.size(); ++i) {
    if (i % 2 == 0) {
      ASSERT_EQ(rcvd_msgs[0]->data[i], 0x55) << "Got unexpected payload value on msg 1, byte " << i;
    } else {
      ASSERT_EQ(rcvd_msgs[0]->data[i], 0xAA) << "Got unexpected payload value on msg 1, byte " << i;
    }
  }

  // MSG 2
  ASSERT_EQ(rcvd_msgs[1]->id, 0x555) << "Got incorrect message ID on msg 2.";
  ASSERT_EQ(rcvd_msgs[1]->dlc, 0) << "Got incorrect DLC on msg 2.";
  ASSERT_FALSE(rcvd_msgs[1]->is_rtr) << "Got unexptected is_rtr value on msg 2.";
  ASSERT_FALSE(rcvd_msgs[1]->is_extended) << "Got unexpected is_extended value on msg 2.";
  ASSERT_FALSE(rcvd_msgs[1]->is_error) << "Got unexpected is_error value on msg 2.";

  for (auto i = 0; i < rcvd_msgs[1]->data.size(); ++i) {
    ASSERT_EQ(rcvd_msgs[1]->data[i], 0) << "Got unexpected payload value on msg 2, byte " << i;
  }

  // MSG 3
  ASSERT_EQ(rcvd_msgs[2]->id, 0x555) << "Got incorrect message ID on msg 3.";
  ASSERT_EQ(rcvd_msgs[2]->dlc, 8) << "Got incorrect DLC on msg 3.";
  ASSERT_FALSE(rcvd_msgs[2]->is_rtr) << "Got unexptected is_rtr value on msg 3.";
  ASSERT_TRUE(rcvd_msgs[2]->is_extended) << "Got unexpected is_extended value on msg 3.";
  ASSERT_FALSE(rcvd_msgs[2]->is_error) << "Got unexpected is_error value on msg 3.";

  for (auto i = 0; i < rcvd_msgs[2]->data.size(); ++i) {
    if (i % 2 == 0) {
      ASSERT_EQ(rcvd_msgs[2]->data[i], 0x55) << "Got unexpected payload value on msg 3, byte " << i;
    } else {
      ASSERT_EQ(rcvd_msgs[2]->data[i], 0xAA) << "Got unexpected payload value on msg 3, byte " << i;
    }
  }

  // MSG 4
  ASSERT_EQ(rcvd_msgs[3]->id, 0x15555555) << "Got incorrect message ID on msg 4.";
  ASSERT_EQ(rcvd_msgs[3]->dlc, 8) << "Got incorrect DLC on msg 4.";
  ASSERT_FALSE(rcvd_msgs[3]->is_rtr) << "Got unexptected is_rtr value on msg 4.";
  ASSERT_TRUE(rcvd_msgs[3]->is_extended) << "Got unexpected is_extended value on msg 4.";
  ASSERT_FALSE(rcvd_msgs[3]->is_error) << "Got unexpected is_error value on msg 4.";

  for (auto i = 0; i < rcvd_msgs[3]->data.size(); ++i) {
    if (i % 2 == 0) {
      ASSERT_EQ(rcvd_msgs[3]->data[i], 0x55) << "Got unexpected payload value on msg 4, byte " << i;
    } else {
      ASSERT_EQ(rcvd_msgs[3]->data[i], 0xAA) << "Got unexpected payload value on msg 4, byte " << i;
    }
  }
}

void reader_callback(const can_msgs::Frame::ConstPtr & msg)
{
  rcvd_msgs.push_back(msg);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "Kvaser Interface Testing Node");
  ros::NodeHandle nh;

  auto msg_pub = nh.advertise<can_msgs::Frame>("/writer3/can_rx", 20);
  auto msg_sub = nh.subscribe("/reader3/can_tx", 20, reader_callback);

  // Wait until reader and writer are ready
  while (true) {
    if (msg_pub.getNumSubscribers() < 1 &&
      msg_sub.getNumPublishers() < 1)
    {
      ros::Duration(0.1).sleep();
    } else {
      break;
    }
  }

  can_msgs::Frame can_msg;

  can_msg.is_rtr = false;
  can_msg.is_extended = false;
  can_msg.is_error = false;

  // MSG 1
  // (VALID) Standard ID and DLC matches payload
  can_msg.header.seq = 0;
  can_msg.id = 0x555;
  can_msg.dlc = 8;

  for (auto i = 0; i < can_msg.dlc; ++i) {
    can_msg.data[i] = (i % 2 == 0) ? 0x55 : 0xAA;
  }

  msg_pub.publish(can_msg);

  // MSG 2
  // (VALID) Standard ID and DLC does not match payload
  // Because can_msgs/Frame->data is fixed length, payload bytes are ignored.
  can_msg.header.seq = 1;
  can_msg.dlc = 0;

  msg_pub.publish(can_msg);

  // MSG 3
  // (VALID) Standard ID but is_extended = true
  // Standard IDs are a subset of extended IDs so still valid.
  can_msg.header.seq = 2;
  can_msg.is_extended = true;
  can_msg.dlc = 8;

  msg_pub.publish(can_msg);

  // MSG 4
  // (VALID) Extended ID and is_extended = true
  can_msg.header.seq = 3;
  can_msg.id = 0x15555555;

  msg_pub.publish(can_msg);

  // MSG 5
  // (INVALID) Extended ID and is_extended = false
  // Should not be received (write failure)
  can_msg.header.seq = 4;
  can_msg.is_extended = false;

  msg_pub.publish(can_msg);

  auto later = ros::Time::now() + ros::Duration(3.0);

  // Spin for 3 seconds, then run tests
  while (later > ros::Time::now()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return RUN_ALL_TESTS();
}
