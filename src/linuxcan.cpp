/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <kvaser_interface.h>

using namespace AS::CAN;

//Default constructor.
KvaserCan::KvaserCan() :
  handle(new int)
{
  *handle = -1;
  canInitializeLibrary();
}

//Default destructor.
KvaserCan::~KvaserCan()
{
  if (*handle > -1)
  {
    canClose(*handle);
  }
}

return_statuses KvaserCan::open(const int& hardware_id,
                                const int& circuit_id,
                                const int& bitrate,
                                const bool& echo_on)
{
  if (*handle < 0)
  {
    return INIT_FAILED;
  }

  if (!on_bus)
  {
    int numChan;

    if (canGetNumberOfChannels(&numChan) != canOK)
    {
      return INIT_FAILED;
    }

    unsigned int serial[2];
    unsigned int channel_number;
    int channel = -1;

    for (int idx = 0; idx < numChan; idx++)
    {
      if (canGetChannelData(idx, canCHANNELDATA_CARD_SERIAL_NO, &serial, sizeof(serial)) == canOK)
      {
        if (serial[0] == (unsigned int) hardware_id)
        {
          if (canGetChannelData(idx, canCHANNELDATA_CHAN_NO_ON_CARD, &channel_number, sizeof(channel_number)) == canOK)
          {
            if (channel_number == (unsigned int) circuit_id)
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
    {
      return INIT_FAILED;
    }

    // Set bit rate and other parameters
    long freq;

    switch (bitrate)
    {
      case 125000:
        freq = canBITRATE_125K;
        break;
      case 250000:
        freq = canBITRATE_250K;
        break;
      case 500000:
        freq = canBITRATE_500K;
        break;
      case 1000000:
        freq = canBITRATE_1M;
        break;
      default:
      {
        return  BAD_PARAM;
      }
    }

    if (canSetBusParams(*handle, freq, 0, 0, 0, 0, 0) < 0)
    {
      return BAD_PARAM;
    }

    // Linuxcan defaults to echo on, so if you've opened the same can channel
    // from multiple interfaces they will receive the messages that each other
    // send.  Turn it off here if desired.
    if (!echo_on)
    {
      unsigned char off = 0;
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

bool KvaserCan::is_open()
{
  if (*handle < 0)
  {
    return false;
  }
  else
  {
    if (on_bus)
    {
      unsigned long int flags;

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

return_statuses KvaserCan::close()
{
  if (*handle < 0)
  {
    return CHANNEL_CLOSED;
  }

  // Close the channel
  if (canClose(*handle) != canOK)
  {
    return CLOSE_FAILED;
  }

  on_bus = false;

  return OK;
}

return_statuses KvaserCan::read(long *id,
                                unsigned char *msg,
                                unsigned int *size,
                                bool *extended,
                                unsigned long *time)
{
  if (*handle < 0)
  {
    return CHANNEL_CLOSED;
  }

  bool done = false;
  return_statuses ret_val = INIT_FAILED;
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
  {
    *extended = ((flag & canMSG_EXT) > 0);
  }

  return ret_val;
}

return_statuses KvaserCan::write(const long& id,
                                 unsigned char *msg,
                                 const unsigned int& size,
                                 const bool& extended)
{
  if (*handle < 0)
  {
    return CHANNEL_CLOSED;
  }

  unsigned int flag;

  if (extended)
  {
    flag = canMSG_EXT;
  }
  else
  {
    flag = canMSG_STD;
  }

  canStatus ret = canWrite(*handle, id, msg, size, flag);

  return (ret == canOK) ? OK : WRITE_FAILED;
}
