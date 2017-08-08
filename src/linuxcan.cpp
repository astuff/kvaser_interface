/*
*   linuxcan.cpp - Kvaser linuxcan implementation for AutonomouStuff generic CAN interface.
*   Copyright (C) 2017 AutonomouStuff, Co.
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <can_interface.h>
#include <canlib.h>

using namespace std;
using namespace AS::CAN;

//Default constructor.
CanInterface::CanInterface() :
        handle(NULL)
{
  handle = malloc(sizeof(canHandle));
}

//Default destructor.
CanInterface::~CanInterface()
{
  if (handle != NULL)
  {
    canHandle *h = (canHandle*) handle;
    canClose(*h);
  }

  free(handle);
}

return_statuses CanInterface::open(int hardware_id, int circuit_id, int bitrate)
{
  if (handle == NULL)
  {
    return INIT_FAILED;
  }

  if (!onBus)
  {
    canHandle *h = (canHandle *) handle;

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
      return BAD_PARAMS;
    }

    // Open channel
    *h = canOpenChannel(channel, canOPEN_ACCEPT_VIRTUAL);
    if (*h < 0)
    {
      return INIT_FAILED;
    }

    // Set bit rate and other parameters
    long freq;
    switch (bitrate)
    {
      case 125000: freq = canBITRATE_125K; break;
      case 250000: freq = canBITRATE_250K; break;
      case 500000: freq = canBITRATE_500K; break;
      case 1000000: freq = canBITRATE_1M; break;
      default:
      {
        return  BAD_PARAMS;
      }
    }

    if (canSetBusParams(*h, freq, 0, 0, 0, 0, 0) < 0)
    {
      return BAD_PARAMS;
    }

    // Set output control
    canSetBusOutputControl(*h, canDRIVER_NORMAL);
    canBusOn(*h);
    onBus = true;
  }

  return OK;
}

return_statuses CanInterface::close()
{
  if (handle == NULL)
  {
    return INIT_FAILED;
  }

  canHandle *h = (canHandle *) handle;

  // Close the channel
  canClose(*h);
  onBus = false;

  return OK;
}

return_statuses CanInterface::read(long *id, unsigned char *msg, unsigned int *size, bool *extended, unsigned long *time)
{
  if (handle == NULL)
  {
    return INIT_FAILED;
  }

  canHandle *h = (canHandle *) handle;

  bool done = false;
  return_statuses ret_val = INIT_FAILED;
  unsigned int flag = 0;

  while (!done)
  {
    canStatus ret = canRead(*h, id, msg, size, &flag, time);

    if (ret == canERR_NOMSG)
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

return_statuses CanInterface::send(long id, unsigned char *msg, unsigned int size, bool extended)
{
  if (handle == NULL)
  {
    return INIT_FAILED;
  }

  canHandle *h = (canHandle *) handle;

  unsigned int flag;

  if (extended)
  {
    flag = canMSG_EXT;
  }
  else
  {
    flag = canMSG_STD;
  }

  canStatus ret = canWrite(*h, id, msg, size, flag);

  return (ret == canOK) ? OK : SEND_FAILED;
}
