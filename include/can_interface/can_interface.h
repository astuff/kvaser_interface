/*
*   can_interface.h - Header file for AutonomouStuff generic CAN interface.
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

// Define a class that supports a basic CAN interface that's independent of the hardware and driver library used
// Different libraries can be created to define all these functions for a specific driver library

#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

//C++ Includes
#include <iostream>

//OS Includes
#include <unistd.h>

namespace AS
{
namespace CAN
{
  enum return_statuses
  {
    OK = 0,
    INIT_FAILED = -1,
    BAD_PARAMS = -2,
    NO_CHANNELS_FOUND = -3,
    CHANNEL_CLOSED = -4,
    NO_MESSAGES_RECEIVED = -5,
    READ_FAILED = -6,
    WRITE_FAILED = -7,
    CLOSE_FAILED = -8
  };

  class CanInterface
  {
  public:
    CanInterface();

    ~CanInterface();

    // Called to pass in parameters and open can link
    return_statuses open(int hardware_id, int circuit_id, int bitrate);

    // Close the can link
    return_statuses close();

    // Read a message
    return_statuses read(long *id, unsigned char *msg, unsigned int *size, bool *extended, unsigned long *time);

    // Send a message
    return_statuses write(long id, unsigned char *msg, unsigned int size, bool extended);

  private:
    void *handle;
    bool on_bus;
  };

  // Converts error messages to human-readable strings
  std::string return_status_desc(return_statuses &ret);
}
}
#endif
