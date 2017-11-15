/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include <kvaser_interface.h>

std::string AS::CAN::return_status_desc(const return_statuses& ret)
{
  std::string status_string;

  if (ret == INIT_FAILED)
  {
    status_string = "Initialization of the CAN interface failed.";
  }
  else if (ret == BAD_PARAM)
  {
    status_string = "A bad parameter was provided to the CAN interface during initalization.";
  }
  else if (ret == NO_CHANNELS_FOUND)
  {
    status_string = "No available CAN channels were found.";
  }
  else if (ret == CHANNEL_CLOSED)
  {
    status_string = "CAN channel is not currently open.";
  }
  else if (ret == NO_MESSAGES_RECEIVED)
  {
    status_string = "No messages were received on the interface.";
  }
  else if (ret == READ_FAILED)
  {
    status_string = "A read operation failed on the CAN interface.";
  }
  else if (ret == WRITE_FAILED)
  {
    status_string = "A write operation failed on the CAN interface.";
  }
  else if (ret == CLOSE_FAILED)
  {
    status_string = "Closing the CAN interface failed.";
  }

  return status_string;
}
