/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <kvaser_interface/kvaser_interface.h>

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

ReturnStatuses KvaserCan::open(const int32_t& hardware_id,
                               const int32_t& circuit_id,
                               const int32_t& bitrate,
                               const bool& echo_on)
{
  if (*handle < 0)
    return INIT_FAILED;

  if (!on_bus)
  {
    int numChan;

    if (canGetNumberOfChannels(&numChan) != canOK)
      return INIT_FAILED;

    uint32_t serial[2];
    uint32_t channel_number;
    int32_t channel = -1;

    for (int32_t idx = 0; idx < numChan; idx++)
    {
      if (canGetChannelData(idx, canCHANNELDATA_CARD_SERIAL_NO, &serial, sizeof(serial)) == canOK)
      {
        if (serial[0] == (uint32_t) hardware_id)
        {
          if (canGetChannelData(idx, canCHANNELDATA_CHAN_NO_ON_CARD, &channel_number, sizeof(channel_number)) == canOK)
          {
            if (channel_number == (uint32_t) circuit_id)
            {
              channel = idx;
            }
          }
        }
      }
    }

    if (channel == -1)
    {
      return BAD_PARAM;
    }

    // Open channel
    *handle = canOpenChannel(channel, canOPEN_ACCEPT_VIRTUAL);

    if (*handle < 0)
      return INIT_FAILED;

    // Set bit rate and other parameters
    int64_t freq;

    switch (bitrate)
    {
      case 125000: freq = canBITRATE_125K; break;
      case 250000: freq = canBITRATE_250K; break;
      case 500000: freq = canBITRATE_500K; break;
      case 1000000: freq = canBITRATE_1M; break;
      default: return  BAD_PARAM;
    }

    if (canSetBusParams(*handle, freq, 0, 0, 0, 0, 0) < 0)
      return BAD_PARAM;

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
      return INIT_FAILED;

    on_bus = true;
  }

  return OK;
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
    return CHANNEL_CLOSED;

  // Close the channel
  if (canClose(*handle) != canOK)
    return CLOSE_FAILED;

  on_bus = false;

  return OK;
}

ReturnStatuses KvaserCan::read(int64_t *id,
                               uint8_t *msg,
                               uint32_t *size,
                               bool *extended,
                               uint64_t *time)
{
  if (*handle < 0)
  {
    return CHANNEL_CLOSED;
  }

  bool done = false;
  ReturnStatuses ret_val = INIT_FAILED;
  unsigned int flag = 0;

  while (!done)
  {
    canStatus ret = canRead(*handle, id, msg, size, &flag, time);

    if (ret == canERR_NOTINITIALIZED)
    {
      ret_val = CHANNEL_CLOSED;
      on_bus = false;
      done = true;
    }
    else if (ret == canERR_NOMSG)
    {
      ret_val = NO_MESSAGES_RECEIVED;
      done = true;
    }
    else if (ret != canOK)
    {
      ret_val = READ_FAILED;
      done = true;
    }
    else if (!(flag & 0xF9))
    {
      // Was a received message with actual data
      ret_val = OK;
      done = true;
    }
    // Else a protocol message, such as a TX ACK, was received
    // Keep looping until one of the other conditions above is met
  }

  if (ret_val == OK)
    *extended = ((flag & canMSG_EXT) > 0);

  return ret_val;
}

ReturnStatuses KvaserCan::write(const int64_t& id,
                                uint8_t *msg,
                                const uint32_t& size,
                                const bool& extended)
{
  if (*handle < 0)
    return CHANNEL_CLOSED;

  uint32_t flag;

  if (extended)
    flag = canMSG_EXT;
  else
    flag = canMSG_STD;

  canStatus ret = canWrite(*handle, id, msg, size, flag);

  return (ret == canOK) ? OK : WRITE_FAILED;
}
