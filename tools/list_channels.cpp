/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "kvaser_interface/kvaser_interface.h"

#include <iostream>
#include <algorithm>

using AS::CAN::KvaserCanUtils;
using AS::CAN::KvaserChannel;

int main(int argc, char ** argv)
{
  auto channels = KvaserCanUtils::getChannels();

  if (channels.size() > 0)
  {
    auto cards = KvaserCanUtils::getCards();

    for (const auto & channel : channels)
    {
      std::cout << "Channel Index: " << channel->channel_idx << ", ";
      std::cout << "Circuit No: " << channel->channel_no_on_card << ", ";
      std::cout << "Card S/N: " << channel->serial_no << ", ";
      std::cout << "Card UPC: " << channel->upc_no << std::endl;
    }
  }
  else
  {
    std::cerr << "No channels found." << std::endl;
    return -1;
  }

  return 0;
}
