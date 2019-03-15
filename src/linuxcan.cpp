/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <kvaser_interface/kvaser_interface.h>

#include <string>
#include <vector>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <iomanip>

using namespace AS::CAN;

KvaserCan::KvaserCan() :
  handle(new int32_t)
{
  *handle = -1;
  canInitializeLibrary();
}

KvaserCan::~KvaserCan()
{
  if (*handle > -1)
    canClose(*handle);
}

ReturnStatuses KvaserCan::open(const uint64_t& hardware_id,
                               const uint32_t& circuit_id,
                               const uint32_t& bitrate,
                               const bool& echo_on)
{
  auto channels = KvaserCanUtils::getChannels();
  uint32_t channel_index = 0;
  bool channel_found = false;

  for (const auto & channel : channels)
  {
    if (hardware_id == channel->serial_no &&
        circuit_id == channel->channel_no_on_card)
    {
      channel_index = channel->channel_idx;
      channel_found = true;
      break;
    }
  }

  if (channel_found)
    return open(channel_index, bitrate, echo_on);
  else
    return ReturnStatuses::BAD_PARAM;
}

ReturnStatuses KvaserCan::open(const uint32_t& channel_index,
                               const uint32_t& bitrate,
                               const bool& echo_on)
{
  if (!on_bus)
  {
    int32_t numChan = -1;
    KvaserCanUtils::getChannelCount(&numChan);

    if (numChan < 0)
      return ReturnStatuses::NO_CHANNELS_FOUND;

    // Open channel
    *handle = canOpenChannel(channel_index, canOPEN_ACCEPT_VIRTUAL);

    if (*handle < 0)
      return ReturnStatuses::INIT_FAILED;

    // Set bit rate and other parameters
    int64_t freq;

    switch (bitrate)
    {
      case 125000: freq = canBITRATE_125K; break;
      case 250000: freq = canBITRATE_250K; break;
      case 500000: freq = canBITRATE_500K; break;
      case 1000000: freq = canBITRATE_1M; break;
      default: return  ReturnStatuses::BAD_PARAM;
    }

    if (canSetBusParams(*handle, freq, 0, 0, 0, 0, 0) < 0)
      return ReturnStatuses::BAD_PARAM;

    // Linuxcan defaults to echo on, so if you've opened the same can channel
    // from multiple interfaces they will receive the messages that each other
    // send.  Turn it off here if desired.
    if (!echo_on)
    {
      uint8_t off = 0;
      canIoCtl(*handle, canIOCTL_SET_LOCAL_TXECHO, &off, 1);
    }

    // Set output control
    canSetBusOutputControl(*handle, canDRIVER_NORMAL);

    if (canBusOn(*handle) < 0)
      return ReturnStatuses::INIT_FAILED;

    on_bus = true;
  }

  return ReturnStatuses::OK;
}

bool KvaserCan::isOpen()
{
  if (*handle < 0)
  {
    return false;
  }
  else
  {
    if (on_bus)
    {
      uint64_t flags;

      canStatus ret = canReadStatus(*handle, &flags);

      if (ret != canOK)
        return false;

      if ((flags & canSTAT_BUS_OFF) > 1)
      {
        close();
        return false;
      }
      else
      {
        return true;
      }
    }
    else
    {
      return false;
    }
  }
}

ReturnStatuses KvaserCan::close()
{
  if (*handle < 0)
    return ReturnStatuses::CHANNEL_CLOSED;

  // Close the channel
  if (canClose(*handle) != canOK)
    return ReturnStatuses::CLOSE_FAILED;

  *handle = -1;
  on_bus = false;

  return ReturnStatuses::OK;
}

ReturnStatuses KvaserCan::read(uint32_t * id,
                               uint8_t * msg,
                               uint32_t * size,
                               bool * extended,
                               uint64_t * time)
{
  if (*handle < 0)
  {
    return ReturnStatuses::CHANNEL_CLOSED;
  }

  bool done = false;
  ReturnStatuses ret_val = ReturnStatuses::INIT_FAILED;
  unsigned int flag = 0;

  while (!done)
  {
    int64_t id_proxy = 0;
    canStatus ret = canRead(*handle, &id_proxy, msg, size, &flag, time);
    *id = static_cast<uint32_t>(id_proxy);

    if (ret == canERR_NOTINITIALIZED)
    {
      ret_val = ReturnStatuses::CHANNEL_CLOSED;
      on_bus = false;
      done = true;
    }
    else if (ret == canERR_NOMSG)
    {
      ret_val = ReturnStatuses::NO_MESSAGES_RECEIVED;
      done = true;
    }
    else if (ret != canOK)
    {
      ret_val = ReturnStatuses::READ_FAILED;
      done = true;
    }
    else if (!(flag & 0xF9))
    {
      // Was a received message with actual data
      ret_val = ReturnStatuses::OK;
      done = true;
    }
    // Else a protocol message, such as a TX ACK, was received
    // Keep looping until one of the other conditions above is met
  }

  if (ret_val == ReturnStatuses::OK)
    *extended = ((flag & canMSG_EXT) > 0);

  return ret_val;
}

ReturnStatuses KvaserCan::write(const uint32_t & id,
                                uint8_t * msg,
                                const uint32_t & size,
                                const bool & extended)
{
  if (*handle < 0)
    return ReturnStatuses::CHANNEL_CLOSED;

  uint32_t flag;

  if (extended)
    flag = canMSG_EXT;
  else
    flag = canMSG_STD;

  canStatus ret = canWrite(*handle, id, msg, size, flag);

  return (ret == canOK) ? ReturnStatuses::OK : ReturnStatuses::WRITE_FAILED;
}

ReturnStatuses KvaserCanUtils::canlibStatToReturnStatus(const int32_t & canlibStat)
{
  switch (canlibStat)
  {
    case canOK:
      return ReturnStatuses::OK;
    case canERR_PARAM:
      return ReturnStatuses::BAD_PARAM;
    case canERR_NOTFOUND:
      return ReturnStatuses::NO_CHANNELS_FOUND;
    default:
      return ReturnStatuses::INIT_FAILED;
  }
}

void KvaserCanUtils::getChannelCount(int32_t * numChan)
{
  auto stat = canGetNumberOfChannels(numChan);

  if (stat != canOK)
    *numChan = -1;
}

std::vector<std::shared_ptr<KvaserCard>> KvaserCanUtils::getCards()
{
  auto channels = getChannels();

  std::vector<std::shared_ptr<KvaserCard>> cards;

  for (const auto & channel : channels)
  {
    bool found = false;

    for (const auto & card : cards)
    {
      if (card->serial_no == channel->serial_no)
        found = true;
    }

    if (!found)
      cards.emplace_back(std::dynamic_pointer_cast<KvaserCard>(std::move(channel)));
  }

  return cards;
}

std::vector<std::shared_ptr<KvaserChannel>> KvaserCanUtils::getChannels()
{
  int32_t numChan = -1;
  std::vector<std::shared_ptr<KvaserChannel>> channels;
  KvaserCanUtils::getChannelCount(&numChan);

  // Sanity checks before continuing
  if (numChan > -1 && numChan < 300)
  {
    for (auto i = 0; i < numChan; ++i)
    {
      KvaserChannel chan;
      int stat = 0;

      chan.channel_idx = i;

      uint64_t serial = 0;
      uint32_t channel_no = 0;
      uint32_t card_type = 0;
      uint16_t firmware_rev[4];
      uint32_t max_bitrate = 0;
      char dev_name[256];
      uint32_t upc_no[2];
      char driver_name[256];
      uint16_t driver_ver[4];

      memset(dev_name, 0, sizeof(dev_name));
      memset(driver_name, 0, sizeof(driver_name));

      stat = canGetChannelData(i, canCHANNELDATA_CARD_SERIAL_NO, &serial, sizeof(serial));

      if (stat == canOK)
        chan.serial_no = serial;
      else
        chan.all_data_valid = false;

      stat = canGetChannelData(i, canCHANNELDATA_CHAN_NO_ON_CARD, &channel_no, sizeof(channel_no));

      if (stat == canOK)
        chan.channel_no_on_card = channel_no;
      else
        chan.all_data_valid = false;

      stat = canGetChannelData(i, canCHANNELDATA_CARD_TYPE, &card_type, sizeof(card_type));

      if (stat == canOK)
        chan.hw_type = static_cast<HardwareType>(card_type);
      else
        chan.all_data_valid = false;

      stat = canGetChannelData(i, canCHANNELDATA_CARD_FIRMWARE_REV, &firmware_rev, sizeof(firmware_rev));

      if (stat == canOK)
      {
        chan.firmware_rev_maj = firmware_rev[3];
        chan.firmware_rev_min = firmware_rev[2];
        chan.firmware_rev_rel = firmware_rev[1];
        chan.firmware_rev_bld = firmware_rev[0];
      }
      else
      {
        chan.all_data_valid = false;
      }

      stat = canGetChannelData(i, canCHANNELDATA_MAX_BITRATE, &max_bitrate, sizeof(max_bitrate));

      if (stat == canOK)
        chan.max_bitrate = max_bitrate;
      else
        chan.all_data_valid = false;

      stat = canGetChannelData(i, canCHANNELDATA_DEVDESCR_ASCII, &dev_name, sizeof(dev_name));

      if (stat == canOK)
        chan.dev_name = std::string(dev_name);
      else
        chan.all_data_valid = false;

      stat = canGetChannelData(i, canCHANNELDATA_CARD_UPC_NO, &upc_no, sizeof(upc_no));

      if (stat == canOK)
      {
        std::ostringstream oss;
        oss << std::hex << (upc_no[1] >> 12) << "-";
        oss << (((upc_no[1] & 0xfff) << 8) | ((upc_no[0] >> 24) & 0xff)) << "-";
        oss << std::setfill('0') << std::setw(5) << ((upc_no[0] >> 4) & 0xfffff) << "-";
        oss << (upc_no[0] & 0x0f);
        chan.upc_no = oss.str();
      }
      else
      {
        chan.all_data_valid = false;
      }

      stat = canGetChannelData(i, canCHANNELDATA_DRIVER_NAME, &driver_name, sizeof(driver_name));

      if (stat == canOK)
        chan.driver_name = std::string(driver_name);
      else
        chan.all_data_valid = false;

      stat = canGetChannelData(i, canCHANNELDATA_DLL_FILE_VERSION, &driver_ver, sizeof(driver_ver));

      if (stat == canOK)
      {
        chan.driver_ver_maj = driver_ver[3];
        chan.driver_ver_min = driver_ver[2];
        chan.driver_ver_bld = driver_ver[1];
      }
      else
      {
        chan.all_data_valid = false;
      }

      channels.push_back(std::make_shared<KvaserChannel>(std::move(chan)));
    }
  }

  return channels;
}

std::vector<std::shared_ptr<KvaserChannel>> KvaserCanUtils::getChannelsOnCard(const uint64_t & serialNo)
{
  std::vector<std::shared_ptr<KvaserChannel>> channelsOnCard;

  auto channels = getChannels();

  for (const auto & channel : channels)
  {
    if (channel->serial_no == serialNo)
      channelsOnCard.emplace_back(std::move(channel));
  }

  return channelsOnCard;
}

std::string KvaserCanUtils::returnStatusDesc(const ReturnStatuses& ret)
{
  std::string status_string;

  if (ret == ReturnStatuses::INIT_FAILED)
    status_string = "Initialization of the CAN interface failed.";
  else if (ret == ReturnStatuses::BAD_PARAM)
    status_string = "A bad parameter was provided to the CAN interface during initalization.";
  else if (ret == ReturnStatuses::NO_CHANNELS_FOUND)
    status_string = "No available CAN channels were found.";
  else if (ret == ReturnStatuses::CHANNEL_CLOSED)
    status_string = "CAN channel is not currently open.";
  else if (ret == ReturnStatuses::NO_MESSAGES_RECEIVED)
    status_string = "No messages were received on the interface.";
  else if (ret == ReturnStatuses::READ_FAILED)
    status_string = "A read operation failed on the CAN interface.";
  else if (ret == ReturnStatuses::WRITE_FAILED)
    status_string = "A write operation failed on the CAN interface.";
  else if (ret == ReturnStatuses::CLOSE_FAILED)
    status_string = "Closing the CAN interface failed.";

  return status_string;
}
