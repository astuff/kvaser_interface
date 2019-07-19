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

KvaserCan *KvaserReadCbProxy::kvCanObj = nullptr;
std::shared_ptr<CanHandle> KvaserReadCbProxy::handle = std::shared_ptr<CanHandle>(nullptr);

KvaserCan::KvaserCan() :
  handle(new CanHandle)
{
  *handle = -1;
  canInitializeLibrary();
}

KvaserCan::~KvaserCan()
{
  if (*handle > -1)
    canClose(*handle);
}

ReturnStatuses KvaserCan::open(const uint64_t &hardware_id,
                               const uint32_t &circuit_id,
                               const uint32_t &bitrate,
                               const bool &echo_on)
{
  auto channels = KvaserCanUtils::getChannels();
  uint32_t channel_index = 0;
  bool channel_found = false;

  if (channels.size() < 1)
    return ReturnStatuses::NO_CHANNELS_FOUND;

  for (const auto &channel : channels)
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

ReturnStatuses KvaserCan::open(const uint32_t &channel_index,
                               const uint32_t &bitrate,
                               const bool &echo_on)
{
  if (!on_bus)
  {
    int32_t numChan = -1;
    KvaserCanUtils::getChannelCount(&numChan);

    if (numChan < 1)
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
    return false;
  else
    return on_bus;
}

ReturnStatuses KvaserCan::close()
{
  if (*handle < 0)
    return ReturnStatuses::CHANNEL_CLOSED;

  // Close the channel
  canStatus ret = canClose(*handle);

  *handle = -1;
  on_bus = false;

  if (ret != canOK)
    return ReturnStatuses::CLOSE_FAILED;
  else
    return ReturnStatuses::OK;
}

ReturnStatuses KvaserCan::read(CanMsg *msg)
{
  if (*handle < 0)
  {
    return ReturnStatuses::CHANNEL_CLOSED;
  }

  // Make sure the incoming message is empty
  msg->id = 0;
  msg->dlc = 0;
  msg->flags.clear();
  msg->error_flags.clear();
  msg->data.clear();
  msg->timestamp = 0;

  int64_t id_proxy = 0;
  uint32_t flags = 0;
  char data[64];

  canStatus ret = canRead(*handle, &id_proxy, data, &msg->dlc, &flags, &msg->timestamp);

  msg->id = static_cast<uint32_t>(id_proxy);

  // Only process payload if dlc != 0
  if (msg->dlc != 0)
  {
    size_t bytes = KvaserCanUtils::dlcToSize(msg->dlc);

    msg->data.reserve(bytes);

    for (uint8_t i = 0; i < bytes; ++i)
    {
      msg->data.emplace_back(std::move(data[i]));
    }
  }

  KvaserCanUtils::setMsgFromFlags(msg, flags);

  switch (ret)
  {
    case canOK:
      return ReturnStatuses::OK;
      break;
    case canERR_NOTINITIALIZED:
      on_bus = false;
      *handle = -1;
      return ReturnStatuses::CHANNEL_CLOSED;
      break;
    case canERR_NOMSG:
      return ReturnStatuses::NO_MESSAGES_RECEIVED;
      break;
    default:
      return ReturnStatuses::READ_FAILED;
  }
}

ReturnStatuses KvaserCan::registerReadCallback(std::function<void(void)> callable)
{
  if (!on_bus)
  {
    return ReturnStatuses::CHANNEL_CLOSED;
  }
  else
  {
    readFunc = callable;
    auto ret = KvaserReadCbProxy::registerCb(this, handle);

    if (ret != ReturnStatuses::OK)
    {
      readFunc = nullptr;
    }

    return ret;
  }
}

ReturnStatuses KvaserCan::write(CanMsg &&msg)
{
  if (*handle < 0)
    return ReturnStatuses::CHANNEL_CLOSED;

  // DLC to Payload Size Check
  auto payload_size = KvaserCanUtils::dlcToSize(msg.dlc);

  if (payload_size != msg.data.size())
    return ReturnStatuses::DLC_PAYLOAD_MISMATCH;

  uint32_t flags = 0;
  KvaserCanUtils::setFlagsFromMsg(msg, &flags);

  canStatus ret = canWrite(*handle, msg.id, &msg.data[0], msg.dlc, flags);

  return (ret == canOK) ? ReturnStatuses::OK : ReturnStatuses::WRITE_FAILED;
}

ReturnStatuses KvaserReadCbProxy::registerCb(KvaserCan *canObj, const std::shared_ptr<CanHandle> &hdl)
{
  handle = std::shared_ptr<CanHandle>(hdl);

  if (!canObj->isOpen())
    return ReturnStatuses::CHANNEL_CLOSED;
  else
  {
    kvCanObj = canObj;

    auto stat = canSetNotify(
      *(hdl),
      &KvaserReadCbProxy::proxyCallback,
      canNOTIFY_RX,
      reinterpret_cast<char*>(0));

    if (stat == canOK)
    {
      return ReturnStatuses::OK;
    }
    else
    {
      kvCanObj = nullptr;
      return ReturnStatuses::BAD_PARAM;
    }
  }
}

void KvaserReadCbProxy::proxyCallback(canNotifyData *data)
{
  kvCanObj->readFunc();
}

ReturnStatuses KvaserCanUtils::canlibStatToReturnStatus(const int32_t &canlibStat)
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

size_t KvaserCanUtils::dlcToSize(const uint8_t &dlc)
{
  if (dlc < 9)
  {
    return dlc;
  }
  else
  {
    switch (dlc)
    {
      case 9:
        return 12;
        break;
      case 10:
        return 16;
        break;
      case 11:
        return 20;
        break;
      case 12:
        return 24;
        break;
      case 13:
        return 32;
        break;
      case 14:
        return 48;
        break;
      case 15:
        return 64;
        break;
      default:
        return 0;
    }
  }
}

uint8_t KvaserCanUtils::sizeToDlc(const size_t &size)
{
  if (size < 9)
  {
    return size;
  }
  else
  {
    switch (size)
    {
      case 12:
        return 9;
        break;
      case 16:
        return 10;
        break;
      case 20:
        return 11;
        break;
      case 24:
        return 12;
        break;
      case 32:
        return 13;
        break;
      case 48:
        return 14;
        break;
      case 64:
        return 15;
        break;
      default:
        return 0;
    }
  }
}

void KvaserCanUtils::getChannelCount(int32_t *numChan)
{
  auto stat = canGetNumberOfChannels(numChan);

  if (stat != canOK)
    *numChan = -1;
}

std::vector<std::shared_ptr<KvaserCard>> KvaserCanUtils::getCards()
{
  auto channels = getChannels();

  std::vector<std::shared_ptr<KvaserCard>> cards;

  for (const auto &channel : channels)
  {
    bool found = false;

    for (const auto &card : cards)
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

std::vector<std::shared_ptr<KvaserChannel>> KvaserCanUtils::getChannelsOnCard(const uint64_t &serialNo)
{
  std::vector<std::shared_ptr<KvaserChannel>> channelsOnCard;

  auto channels = getChannels();

  for (const auto &channel : channels)
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

void KvaserCanUtils::setMsgFromFlags(CanMsg *msg, const uint32_t &flags)
{
  // Regular CAN message flags
  msg->flags.rtr = ((flags & canMSG_RTR) > 0);
  msg->flags.std_id = ((flags & canMSG_STD) > 0);
  msg->flags.ext_id = ((flags & canMSG_EXT) > 0);
  msg->flags.wakeup_mode = ((flags & canMSG_WAKEUP) > 0);
  msg->flags.nerr_active = ((flags & canMSG_NERR) > 0);
  msg->flags.error_frame = ((flags & canMSG_ERROR_FRAME) > 0);
  msg->flags.tx_ack = ((flags & canMSG_TXACK) > 0);
  msg->flags.tx_rq = ((flags & canMSG_TXRQ) > 0);
  msg->flags.msg_delayed = ((flags & canMSG_DELAY_MSG) > 0);
  msg->flags.single_shot = ((flags & canMSG_SINGLE_SHOT) > 0);
  msg->flags.tx_nack = ((flags & canMSG_TXNACK) > 0);
  msg->flags.arb_lost = ((flags & canMSG_ABL) > 0);

  // CAN FD flags
  msg->flags.fd_msg = ((flags & canFDMSG_FDF) > 0);
  msg->flags.fd_bitrate_switch = ((flags & canFDMSG_BRS) > 0);
  msg->flags.fd_sndr_err_pass_md = ((flags & canFDMSG_ESI) > 0);

  // Error flags
  msg->error_flags.has_err = ((flags & canMSGERR_MASK) > 0);
  msg->error_flags.hw_overrun_err = ((flags & canMSGERR_HW_OVERRUN) > 0);
  msg->error_flags.sw_overrun_err = ((flags & canMSGERR_SW_OVERRUN) > 0);
  msg->error_flags.stuff_err = ((flags & canMSGERR_STUFF) > 0);
  msg->error_flags.form_err = ((flags & canMSGERR_FORM) > 0);
  msg->error_flags.crc_err = ((flags & canMSGERR_CRC) > 0);
  msg->error_flags.bit0_err = ((flags & canMSGERR_BIT0) > 0);
  msg->error_flags.bit1_err = ((flags & canMSGERR_BIT1) > 0);
  msg->error_flags.any_overrun_err = ((flags & canMSGERR_OVERRUN) > 0);
  msg->error_flags.any_bit_err = ((flags & canMSGERR_BIT) > 0);
  msg->error_flags.any_rx_err = ((flags & canMSGERR_BUSERR) > 0);
}

void KvaserCanUtils::setFlagsFromMsg(const CanMsg &msg, uint32_t *flags)
{
  // Regular CAN message flags
  msg.flags.rtr ? *flags |= canMSG_RTR : *flags &= ~canMSG_RTR;
  msg.flags.std_id ? *flags |= canMSG_STD : *flags &= ~canMSG_STD;
  msg.flags.ext_id ? *flags |= canMSG_EXT : *flags &= ~canMSG_EXT;
  msg.flags.wakeup_mode ? *flags |= canMSG_WAKEUP : *flags &= ~canMSG_WAKEUP;
  msg.flags.nerr_active ? *flags |= canMSG_NERR : *flags &= ~canMSG_NERR;
  msg.flags.error_frame ? *flags |= canMSG_ERROR_FRAME : *flags &= ~canMSG_ERROR_FRAME;
  msg.flags.tx_ack ? *flags |= canMSG_TXACK : *flags &= ~canMSG_TXACK;
  msg.flags.tx_rq ? *flags |= canMSG_TXRQ : *flags &= ~canMSG_TXRQ;
  msg.flags.msg_delayed ? *flags |= canMSG_DELAY_MSG : *flags &= ~canMSG_DELAY_MSG;
  msg.flags.single_shot ? *flags |= canMSG_SINGLE_SHOT : *flags &= ~canMSG_SINGLE_SHOT;
  msg.flags.tx_nack ? *flags |= canMSG_TXNACK : *flags &= ~canMSG_TXNACK;
  msg.flags.arb_lost ? *flags |= canMSG_ABL : *flags &= ~canMSG_ABL;

  // CAN FD *flags
  msg.flags.fd_msg ? *flags |= canFDMSG_FDF : *flags &= ~canFDMSG_FDF;
  msg.flags.fd_bitrate_switch ? *flags |= canFDMSG_BRS : *flags &= ~canFDMSG_BRS;
  msg.flags.fd_sndr_err_pass_md ? *flags |= canFDMSG_ESI : *flags &= ~canFDMSG_ESI;

  // Error *flags
  msg.error_flags.hw_overrun_err ? *flags |= canMSGERR_HW_OVERRUN : *flags &= ~canMSGERR_HW_OVERRUN;
  msg.error_flags.sw_overrun_err ? *flags |= canMSGERR_SW_OVERRUN : *flags &= ~canMSGERR_SW_OVERRUN;
  msg.error_flags.stuff_err ? *flags |= canMSGERR_STUFF : *flags &= ~canMSGERR_STUFF;
  msg.error_flags.form_err ? *flags |= canMSGERR_FORM : *flags &= ~canMSGERR_FORM;
  msg.error_flags.crc_err ? *flags |= canMSGERR_CRC : *flags &= ~canMSGERR_CRC;
  msg.error_flags.bit0_err ? *flags |= canMSGERR_BIT0 : *flags &= ~canMSGERR_BIT0;
  msg.error_flags.bit1_err ? *flags |= canMSGERR_BIT1 : *flags &= ~canMSGERR_BIT1;
}
