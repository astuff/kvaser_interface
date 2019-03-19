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

void shutdown_with_error(const ReturnStatuses &ret, const int &error_num)
{
  std::cerr << static_cast<int>(ret);
  std::cerr << " - " << KvaserCanUtils::returnStatusDesc(ret).c_str();
  std::cerr << std::endl;
  kv_can.close();
  exit(error_num);
}

void sig_handler(int s)
{
  std::cout << "Caught signal " << s << std::endl;
  kv_can.close();
  exit(1);
}

void can_read()
{
  // Read CAN data
  auto ret = ReturnStatuses::OK;

  ret = kv_can.open(channel_idx, bitrate);

  if (ret == ReturnStatuses::OK)
  {
    while (true)
    {
      CanMsg msg;

      auto unix_timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count();

      std::cout << "[" << std::dec << unix_timestamp_ms << "] ";

      if (msg.flags.error_frame)
      {
        std::cout << "ERROR FRAME" << std::endl;
      }
      else if (msg.dlc > 0)
      {
        // Write the message to stdout
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
        shutdown_with_error(ret, -4);
      }
    }
  }
  else
  {
    std::cerr << "Error opening Kvaser interface: ";
    shutdown_with_error(ret, -3);
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

  ReturnStatuses ret;

  ret = kv_can.open(channel_idx, bitrate);

  if (ret == ReturnStatuses::OK)
  {
    ret = kv_can.registerReadCallback(std::function<void()>(can_read));

    if (ret != ReturnStatuses::OK)
    {
      std::cerr << "Error registering reader callback: ";
      shutdown_with_error(ret, -1);
    }
  }
  else
  {
    std::cerr << "Error opening Kvaser interface: ";
    shutdown_with_error(ret, -2);
  }

  std::getchar();

  return 0;
}
