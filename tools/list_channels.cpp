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

#include "kvaser_interface/kvaser_interface.hpp"

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
