/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "kvaser_interface/kvaser_interface.h"
#include "kvaser_interface/cxxopts.h"

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <signal.h>

using AS::CAN::KvaserCanUtils;
using AS::CAN::KvaserChannel;
using AS::CAN::KvaserCan;
using AS::CAN::ReturnStatuses;
using AS::CAN::CanMsg;

KvaserCan kv_can;
uint32_t channel_idx = 0;
uint32_t bitrate = 500000;

void shutdown(
  const ReturnStatuses &ret = ReturnStatuses::OK,
  const int &error_num = 0)
{
  if (ret != ReturnStatuses::OK || error_num != 0)
  {
    std::cerr << static_cast<int>(ret);
    std::cerr << " - " << KvaserCanUtils::returnStatusDesc(ret).c_str();
    std::cerr << std::endl;
  }

  kv_can.close();
  exit(error_num);
}

void sig_handler(int s)
{
  std::cout << "Caught signal " << s << std::endl;
  shutdown();
}

void can_read()
{
  auto ret = ReturnStatuses::OK;
  size_t unix_timestamp_ms;

  ret = kv_can.open(channel_idx, bitrate);

  // Read CAN data if channel is OK
  if (ret == ReturnStatuses::OK)
  {
    // Loop until ReturnStatus is NO_MESSAGES_RECEIVED
    while (true)
    {
      CanMsg msg;
      ret = kv_can.read(&msg);

      unix_timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count();

      if (msg.flags.error_frame)
      {
        std::cout << "[" << std::dec << unix_timestamp_ms << "] ";
        std::cout << "ERROR FRAME" << std::endl;
      }
      else if (msg.dlc > 0)
      {
        // Write the message to stdout
        std::cout << "[" << std::dec << unix_timestamp_ms << "] ";
        std::cout << "ID 0x" << std::hex << std::uppercase << msg.id << ":";
        std::cout << std::internal << std::setfill('0');

        for (const auto byte : msg.data)
        {
          std::cout << " " << std::hex << std::setw(2);
          std::cout << std::uppercase << static_cast<unsigned int>(byte);
        }

        std::cout << std::endl;
      }

      if (ret == ReturnStatuses::NO_MESSAGES_RECEIVED)
      {
        break;
      }
      else if (ret != ReturnStatuses::OK)
      {
        std::cerr << "Error reading CAN message: ";
        shutdown(ret, -5);
      }
    }

    ret = kv_can.close();

    if (ret != ReturnStatuses::OK)
    {
      std::cerr << "Error closing Kvaser interface after read: ";
      shutdown(ret, -4);
    }
  }
  else
  {
    std::cerr << "Error opening Kvaser interface: ";
    shutdown(ret, -3);
  }
}

int main(int argc, char ** argv)
{
  // Catch CTRL+C
  struct sigaction sigIntHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Parse options
  cxxopts::Options options("canmonitor", "A simple tool for reading data from a CAN channel.");

  options.add_options()
    ("i, index", "Channel index", cxxopts::value<unsigned int>()->implicit_value("0")->default_value("0"))
    ("b, bitrate", "Bitrate", cxxopts::value<unsigned int>()->implicit_value("500000")->default_value("500000"))
    ("h, help", "Print help", cxxopts::value<bool>()->implicit_value("true")->default_value("false"));

  auto result = options.parse(argc, argv);

  channel_idx = result["index"].as<unsigned int>();
  bitrate = result["bitrate"].as<unsigned int>();

  if (channel_idx > 300 ||
      bitrate < 100 ||
      bitrate > 8000000 ||
      result["help"].as<bool>())
  {
    std::cout << std::endl;
    std::cout << options.help();
    std::cout << std::endl;
    return 0;
  }

  while (true)
  {
    // Below is an example for one method of reading from a CAN
    // channel using a KvaserCan object. However, if better performance
    // is required, registering a read callback function is preferred.
    can_read();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
