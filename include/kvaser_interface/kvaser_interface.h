/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the Kvaser ROS driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#ifndef KVASER_INTERFACE_KVASER_INTERFACE_H
#define KVASER_INTERFACE_KVASER_INTERFACE_H

// C++ Includes
#include <iostream>
#include <memory>

extern "C"
{
#include <canlib.h>
}

namespace AS
{
namespace CAN
{

enum ReturnStatuses
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
    ReturnStatuses open(const int32_t& hardware_id,
                        const int32_t& circuit_id,
                        const int32_t& bitrate,
                        const bool& echo_on = true);

    // Close the can link
    ReturnStatuses close();

    // Check to see if the CAN link is open
    bool isOpen();

    // Read a message
    ReturnStatuses read(int64_t *id,
                        uint8_t *msg,
                        uint32_t *size,
                        bool *extended,
                        uint64_t *time);

    // Send a message
    ReturnStatuses write(const int64_t& id,
                         uint8_t *msg,
                         const uint32_t& size,
                         const bool& extended);

  private:
    std::unique_ptr<CanHandle> handle;
    bool on_bus;
};

// Converts error messages to human-readable strings
std::string returnStatusDesc(const ReturnStatuses& ret);

}  // namespace CAN
}  // namespace AS

#endif  // KVASER_INTERFACE_KVASER_INTERFACE_H
