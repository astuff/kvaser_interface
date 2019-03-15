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
  auto cards = KvaserCanUtils::getCards();

  if (cards.size() > 0)
  {
    std::cout << std::endl;

    for (const auto & card : cards)
    {
      std::cout << "Card " << (&card - &cards[0]) << ":" << std::endl;
      std::cout << "  S/N: " << card->serial_no << std::endl;
      std::cout << "  UPC: " << card->upc_no << std::endl;
      std::cout << "  Name: " << card->dev_name << std::endl;
      std::cout << "  Firmware rev: v";
        std::cout << card->firmware_rev_maj << ".";
        std::cout << card->firmware_rev_min << ".";
        std::cout << card->firmware_rev_bld << std::endl;
      std::cout << "  Driver: " << card->driver_name << " v";
        std::cout << card->driver_ver_maj << ".";
        std::cout << card->driver_ver_min << ".";
        std::cout << card->driver_ver_bld << std::endl;
      std::cout << std::endl;

      auto channels = KvaserCanUtils::getChannelsOnCard(card->serial_no);

      for (const auto & channel : channels)
      {
        std::cout << "  Channel " << channel->channel_no_on_card << ":" << std::endl;
        std::cout << "    Index: " << channel->channel_idx << std::endl;
        std::cout << "    Max Bitrate: " << channel->max_bitrate << std::endl;
      }

      std::cout << std::endl;
    }
  }
  else
  {
    std::cerr << "No channels found." << std::endl;
    return -1;
  }

  return 0;
}
