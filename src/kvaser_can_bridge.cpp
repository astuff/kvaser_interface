/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <thread>
#include <mutex>
#include <chrono>
#include <algorithm>

#include <ros/ros.h>
#include <kvaser_interface/kvaser_interface.h>
#include <can_msgs/Frame.h>

using namespace AS::CAN;

int bit_rate = 500000;
int hardware_id = 0;
int circuit_id = 0;
bool global_keep_going = true;
std::mutex keep_going_mut;
KvaserCan can_reader, can_writer;
ros::Publisher can_tx_pub;

void can_read()
{
  const std::chrono::milliseconds loop_pause = std::chrono::milliseconds(10);
  bool keep_going = true;

  // Set local to global value before looping.
  keep_going_mut.lock();
  keep_going = global_keep_going;
  keep_going_mut.unlock();

  ReturnStatuses ret;

  while (keep_going)
  {
    std::chrono::system_clock::time_point next_time = std::chrono::system_clock::now();
    next_time += loop_pause;

    if (!can_reader.isOpen())
    {
      ret = can_reader.open(hardware_id, circuit_id, bit_rate, false);

      if (ret != ReturnStatuses::OK)
        ROS_ERROR_THROTTLE(0.5, "Kvaser CAN Interface - Error opening reader: %d - %s",
          static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
    }
    else
    {
      while (true)
      {
        CanMsg msg;

        ret = can_reader.read(&msg);

        if (ret  == ReturnStatuses::OK)
        {
          can_msgs::Frame can_pub_msg;
          can_pub_msg.header.frame_id = "0";
          can_pub_msg.id = msg.id;
          can_pub_msg.dlc = msg.data.size();
          can_pub_msg.is_extended = msg.flags.ext_id;
          can_pub_msg.is_error = msg.flags.error_frame;
          can_pub_msg.is_rtr = msg.flags.rtr;
          std::copy(msg.data.begin(), msg.data.end(), can_pub_msg.data.begin());
          can_pub_msg.header.stamp = ros::Time::now();
          can_tx_pub.publish(can_pub_msg);
        }
        else
        {
          break;
        }
      }

      if (ret != ReturnStatuses::NO_MESSAGES_RECEIVED)
        ROS_WARN_THROTTLE(0.5, "Kvaser CAN Interface - Error reading CAN message: %d - %s",
          static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
    }

    std::this_thread::sleep_until(next_time);

    // Set local to global immediately before next loop.
    keep_going_mut.lock();
    keep_going = global_keep_going;
    keep_going_mut.unlock();
  }

  if (can_reader.isOpen())
  {
    ret = can_reader.close();

    if (ret != ReturnStatuses::OK)
      ROS_ERROR_THROTTLE(0.5, "Kvaser CAN Interface - Error closing reader: %d - %s",
        static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
  }
}

void can_rx_callback(const can_msgs::Frame::ConstPtr& msg)
{
  ReturnStatuses ret;

  if (!can_writer.isOpen())
  {
    // Open the channel.
    ret = can_writer.open(hardware_id, circuit_id, bit_rate, false);

    if (ret != ReturnStatuses::OK)
    {
      ROS_ERROR_THROTTLE(0.5, "Kvaser CAN Interface - Error opening writer: %d - %s",
        static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
    }
  }

  if (can_writer.isOpen())
  {
    ret = can_writer.write(msg->id,
                           const_cast<unsigned char*>(&(msg->data[0])),
                           msg->dlc,
                           msg->is_extended);

    if (ret != ReturnStatuses::OK)
      ROS_WARN_THROTTLE(0.5, "Kvaser CAN Interface - CAN send error: %d - %s",
        static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
  }
}

int main(int argc, char** argv)
{
  bool exit = false;

  // ROS initialization
  ros::init(argc, argv, "kvaser_can_bridge");
  ros::NodeHandle n;
  ros::NodeHandle priv("~");
  ros::AsyncSpinner spinner(1);

  can_tx_pub = n.advertise<can_msgs::Frame>("can_tx", 500);

  ros::Subscriber can_rx_sub = n.subscribe("can_rx", 500, can_rx_callback);

  // Wait for time to be valid
  ros::Time::waitForValid();

  if (priv.getParam("can_hardware_id", hardware_id))
  {
    ROS_INFO("Kvaser CAN Interface - Got hardware_id: %d", hardware_id);

    if (hardware_id <= 0)
    {
      ROS_ERROR("Kvaser CAN Interface - CAN hardware ID is invalid.");
      exit = true;
    }
  }

  if (priv.getParam("can_circuit_id", circuit_id))
  {
    ROS_INFO("Kvaser CAN Interface - Got can_circuit_id: %d", circuit_id);

    if (circuit_id < 0)
    {
      ROS_ERROR("Kvaser CAN Interface - Circuit ID is invalid.");
      exit = true;
    }
  }

  if (priv.getParam("can_bit_rate", bit_rate))
  {
    ROS_INFO("Kvaser CAN Interface - Got bit_rate: %d", bit_rate);

    if (bit_rate < 0)
    {
      ROS_ERROR("Kvaser CAN Interface - Bit Rate is invalid.");
      exit = true;
    }
  }

  if (exit)
    return 0;

  // Start CAN receiving thread.
  std::thread can_read_thread(can_read);

  spinner.start();

  ros::waitForShutdown();

  ReturnStatuses ret = can_writer.close();

  if (ret != ReturnStatuses::OK)
    ROS_ERROR("Kvaser CAN Interface - Error closing writer: %d - %s",
      static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());

  keep_going_mut.lock();
  global_keep_going = false;
  keep_going_mut.unlock();

  can_read_thread.join();

  return 0;
}
