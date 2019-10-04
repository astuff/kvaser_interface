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

#include "kvaser_interface/kvaser_reader_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "kvaser_interface/ros_utils.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using namespace std::chrono_literals;

namespace AS
{
namespace CAN
{

KvaserReaderNode::KvaserReaderNode(rclcpp::NodeOptions options)
: lc::LifecycleNode("kvaser_reader_node", options)
{
  hardware_id_ = this->declare_parameter("hardware_id", 12345);
  circuit_id_ = this->declare_parameter("circuit_id", 0);
  bit_rate_ = this->declare_parameter("bit_rate", 500000);
  enable_echo_ = this->declare_parameter("enable_echo", false);

  RCLCPP_INFO(this->get_logger(), "Got hardware ID: %d", hardware_id_);
  RCLCPP_INFO(this->get_logger(), "Got circuit ID: %d", circuit_id_);
  RCLCPP_INFO(this->get_logger(), "Got bit rate: %d", bit_rate_);
  RCLCPP_INFO(this->get_logger(), "Message echo is %s", enable_echo_ ? "enabled" : "disabled");
}

LNI::CallbackReturn KvaserReaderNode::on_configure(const lc::State & state)
{
  (void)state;
  ReturnStatuses ret;

  if ((ret = can_reader_.open(hardware_id_, circuit_id_, bit_rate_, enable_echo_)) ==
    ReturnStatuses::OK)
  {
    RCLCPP_DEBUG(this->get_logger(), "Reader successfully configured.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Error opening CAN reader: %d - %s",
      static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
    return LNI::CallbackReturn::FAILURE;
  }

  frames_pub_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 500);
  read_timer_ = this->create_wall_timer(10ms, std::bind(&KvaserReaderNode::read, this));

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserReaderNode::on_activate(const lc::State & state)
{
  (void)state;
  frames_pub_->on_activate();
  RCLCPP_DEBUG(this->get_logger(), "Reader activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserReaderNode::on_deactivate(const lc::State & state)
{
  (void)state;
  frames_pub_->on_deactivate();
  RCLCPP_DEBUG(this->get_logger(), "Reader deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserReaderNode::on_cleanup(const lc::State & state)
{
  (void)state;
  read_timer_.reset();
  frames_pub_.reset();
  RCLCPP_DEBUG(this->get_logger(), "Reader cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserReaderNode::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Reader shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void KvaserReaderNode::read()
{
  ReturnStatuses ret;

  while (true) {
    if (!can_reader_.isOpen()) {
      RCLCPP_ERROR(this->get_logger(), "Tried to read CAN message but reader was closed.");
      return;
    }

    CanMsg msg;
    ret = can_reader_.read(&msg);

    if (ret == ReturnStatuses::OK) {
      // Only publish if msg is not CAN FD,
      // a wakeup message, a transmit acknowledgement,
      // a transmit request, a delay notification,
      // or a failed single-shot.
      if (!(msg.flags.fd_msg ||
        msg.flags.wakeup_mode ||
        msg.flags.tx_ack ||
        msg.flags.tx_rq ||
        msg.flags.msg_delayed ||
        msg.flags.tx_nack))
      {
        auto ros_msg = KvaserRosUtils::to_ros_msg(std::move(msg));
        auto ros_msg_ptr = std::unique_ptr<can_msgs::msg::Frame>(&ros_msg);
        frames_pub_->publish(std::move(ros_msg_ptr));
      }
    } else if (ret == ReturnStatuses::NO_MESSAGES_RECEIVED) {
      break;
    } else {
      RCLCPP_WARN(this->get_logger(), "Error reading CAN message: %d - %s",
        static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret));
    }
  }
}

}  // namespace CAN
}  // namespace AS

RCLCPP_COMPONENTS_REGISTER_NODE(AS::CAN::KvaserReaderNode)
