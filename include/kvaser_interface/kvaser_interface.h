/*
* Unpublished Copyright (c) 2009-2017 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

// Define a class that supports a basic CAN interface that's independent of the hardware and driver library used
// Different libraries can be created to define all these functions for a specific driver library

#ifndef KVASER_INTERFACE_HPP
#define KVASER_INTERFACE_HPP

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
    BAD_PARAM = -2,
    NO_CHANNELS_FOUND = -3,
    CHANNEL_CLOSED = -4,
    NO_MESSAGES_RECEIVED = -5,
    READ_FAILED = -6,
    WRITE_FAILED = -7,
    CLOSE_FAILED = -8
  };

  class KvaserCan
  {
  public:
    KvaserCan();

    ~KvaserCan();

    // Called to pass in parameters and open can link
    return_statuses open(const int& hardware_id,
                         const int& circuit_id,
                         const int& bitrate,
                         const bool& echo_on = true);

    // Close the can link
    return_statuses close();

    // Check to see if the CAN link is open
    bool is_open();

    // Read a message
    return_statuses read(long *id,
                         unsigned char *msg,
                         unsigned int *size,
                         bool *extended,
                         unsigned long *time);

    // Send a message
    return_statuses write(const long& id,
                          unsigned char *msg,
                          const unsigned int& size,
                          const bool& extended);

  private:
    void *handle;
    bool on_bus;
  };

  // Converts error messages to human-readable strings
  std::string return_status_desc(const return_statuses& ret);
}
}
#endif
