// Copyright (c) 2019 AutonomouStuff, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef KVASER_INTERFACE__ROS_UTILS_HPP_
#define KVASER_INTERFACE__ROS_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>

#include <algorithm>
#include <memory>

#include "kvaser_interface/kvaser_interface.hpp"

namespace kvaser_interface
{

/// \brief KvaserRosUtils class which provides tools
/// for converting from KvaserInterface classes to ROS msgs.
class KvaserRosUtils final
{
public:
  /// \brief Converts a CanMsg to a can_msgs::msg::Frame.
  /// \param[in] can_msg The CanMsg to convert.
  /// \return A converted can_msgs::msg::Frame.
  static can_msgs::msg::Frame to_ros_msg(const CanMsg & can_msg)
  {
    can_msgs::msg::Frame ros_msg;
    ros_msg.id = can_msg.id;
    ros_msg.dlc = can_msg.dlc;
    ros_msg.is_extended = can_msg.flags.ext_id;
    ros_msg.is_error = can_msg.flags.error_frame;
    ros_msg.is_rtr = can_msg.flags.rtr;

    if (can_msg.data.size() < 9) {
      std::copy(can_msg.data.begin(), can_msg.data.end(), ros_msg.data.begin());
    } else {
      std::copy(can_msg.data.begin(), can_msg.data.begin() + 8, ros_msg.data.begin());
    }

    return ros_msg;
  }

  /// \brief Converts a can_msgs::msg::Frame to a CanMsg.
  /// \param[in] ros_msg The can_msgs::msg::Frame to convert.
  /// \return A converted CanMsg.
  static CanMsg from_ros_msg(const can_msgs::msg::Frame & ros_msg)
  {
    CanMsg can_msg;
    can_msg.id = ros_msg.id;
    can_msg.dlc = ros_msg.dlc;
    can_msg.flags.ext_id = ros_msg.is_extended;
    can_msg.flags.error_frame = ros_msg.is_error;
    can_msg.flags.rtr = ros_msg.is_rtr;

    auto msg_size = KvaserCanUtils::dlcToSize(ros_msg.dlc);

    for (size_t i = 0; i < msg_size; ++i) {
      can_msg.data.push_back(ros_msg.data[i]);
    }

    return can_msg;
  }
};

}  // namespace kvaser_interface

#endif  // KVASER_INTERFACE__ROS_UTILS_HPP_
