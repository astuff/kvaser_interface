/*
*   utils.cpp - Utility functions for AutonomouStuff generic CAN interface.
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
