/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "kvaser_interface/kvaser_interface.h"
#include "kvaser_interface/cxxopts.h"

#include <iostream>
#include <algorithm>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <signal.h>

using AS::CAN::KvaserCanUtils;
using AS::CAN::KvaserChannel;
using AS::CAN::KvaserCan;
using AS::CAN::ReturnStatuses;

KvaserCan kv_can;

void shutdown()
{
  kv_can.close();
}

void sig_handler(int s)
{
  std::cout << "Caught signal " << s << std::endl;
  shutdown();
  exit(1);
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
    ("c, channel", "Channel index", cxxopts::value<unsigned int>()->implicit_value("0")->default_value("0"))
    ("b, bitrate", "Bitrate", cxxopts::value<unsigned int>()->implicit_value("500000")->default_value("500000"))
    ("h, help", "Print help", cxxopts::value<bool>()->implicit_value("true")->default_value("false"));

  auto result = options.parse(argc, argv);

  uint32_t channel_idx = result["channel"].as<unsigned int>();
  uint32_t bitrate = result["bitrate"].as<unsigned int>();

  if (channel_idx > 300 ||
      bitrate < 100 ||
      bitrate > 8000000 ||
      result["help"].as<bool>())
  {
    std::cout << std::endl;
    std::cout << options.help();
    std::cout << std::endl;
    return -1;
  }

  // Read CAN data
  auto ret = ReturnStatuses::OK;

  ret = kv_can.open(channel_idx, bitrate);

  if (ret == ReturnStatuses::OK)
  {
    uint32_t id;
    uint8_t msg[8] = {};  // zero-initialize array
    uint32_t size;
    bool extended;
    uint64_t t;
    uint16_t no_msg_count = 0;

    while (1)
    {
      while ((ret = kv_can.read(&id, msg, &size, &extended, &t)) == ReturnStatuses::OK)
      {
        // Write the message to stdout
        auto unix_timestamp = std::chrono::seconds(std::time(NULL));
        auto unix_timestamp_ms = std::chrono::milliseconds(unix_timestamp).count();

        std::cout << "[" << unix_timestamp_ms << "] ";
        std::cout << "ID: 0x" << std::hex << id;
        std::cout << ":";

        for (uint8_t i = 0; i < size; ++i)
        {
          std::cout << " " << std::hex << static_cast<unsigned int>(msg[i]);
        }

        std::cout << std::endl;
      }

      if (ret == ReturnStatuses::NO_MESSAGES_RECEIVED)
      {
        no_msg_count++;

        if (no_msg_count > 150)
        {
          std::cout << "No messages received." << std::endl;
          no_msg_count = 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      else
      {
        std::cerr << KvaserCanUtils::returnStatusDesc(ret);
        shutdown();
        return -1;
      }
    }
  }
  else
  {
    std::cerr << KvaserCanUtils::returnStatusDesc(ret) << std::endl;
    shutdown();
    return -1;
  }

  return 0;
}
