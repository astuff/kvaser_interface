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

#include "kvaser_interface/kvaser_writer_node.hpp"

#include <lifecycle_msgs/msg/state.hpp>

#include <memory>
#include <string>
#include <utility>

#include "kvaser_interface/ros_utils.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

namespace kvaser_interface
{

KvaserWriterNode::KvaserWriterNode(rclcpp::NodeOptions options)
: lc::LifecycleNode("kvaser_writer_node", options)
{
  hardware_id_ = this->declare_parameter("hardware_id", 12345);
  circuit_id_ = this->declare_parameter("circuit_id", 0);
  bit_rate_ = this->declare_parameter("bit_rate", 500000);
  enable_echo_ = this->declare_parameter("enable_echo", false);

  // CAN FD
  tseg1_ = this->declare_parameter("tseg1", 0);
  tseg2_ = this->declare_parameter("tseg2", 0);
  sjw_ = this->declare_parameter("sjw", 0);
  data_bit_rate_ = this->declare_parameter("data_bit_rate", 2000000);
  is_canfd_ = this->declare_parameter("is_canfd", false);

  RCLCPP_INFO(this->get_logger(), "Got hardware ID: %ld", hardware_id_);
  RCLCPP_INFO(this->get_logger(), "Got circuit ID: %d", circuit_id_);
  RCLCPP_INFO(this->get_logger(), "Got bit rate: %d", bit_rate_);
  RCLCPP_INFO(this->get_logger(), "Message echo is %s", enable_echo_ ? "enabled" : "disabled");

  RCLCPP_INFO(this->get_logger(), "CAN FD is %s", is_canfd_ ? "enabled" : "disabled");
  if (is_canfd_){
    RCLCPP_INFO(this->get_logger(), "Got tseg1 : %d", tseg1_);
    RCLCPP_INFO(this->get_logger(), "Got tseg2 : %d", tseg2_);
    RCLCPP_INFO(this->get_logger(), "Got sjw : %d", sjw_);
  }

}

LNI::CallbackReturn KvaserWriterNode::on_configure(const lc::State & state)
{
  (void)state;
  ReturnStatuses ret;
  if (is_canfd)
      ret = can_reader_.open(hardware_id_, circuit_id_, bit_rate_, data_bit_rate_, tseg1_, tseg2_, sjw_, echo_on_);
  else
      ret = can_reader_.open(hardware_id_, circuit_id_, bit_rate_, enable_echo_);


  if (ret == ReturnStatuses::OK)
  {
    RCLCPP_DEBUG(this->get_logger(), "Writer successfully configured.");
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "Error opening CAN writer: %d - %s",
      static_cast<int>(ret), KvaserCanUtils::returnStatusDesc(ret).c_str());
    return LNI::CallbackReturn::FAILURE;
  }

  if (is_canfd)
    frames_sub_ = this->create_subscription<can_msgs::msg::FrameFd>(
    "can_rx", 500, std::bind(&KvaserWriterNode::frame_callback, this, std::placeholders::_1));
  else
      frames_sub_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_rx", 500, std::bind(&KvaserWriterNode::frame_callback, this, std::placeholders::_1));




  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserWriterNode::on_activate(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Writer activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserWriterNode::on_deactivate(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Writer deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserWriterNode::on_cleanup(const lc::State & state)
{
  (void)state;
  frames_sub_.reset();
  RCLCPP_DEBUG(this->get_logger(), "Writer cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserWriterNode::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Writer shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void KvaserWriterNode::frame_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    if (!can_writer_.isOpen()) {
      RCLCPP_ERROR(this->get_logger(), "Tried to write CAN message but writer was closed.");
      return;
    }

    auto can_msg = KvaserRosUtils::from_ros_msg(*msg);
    can_writer_.write(std::move(can_msg));
  } else {
    RCLCPP_WARN(this->get_logger(), "Message received but node is not active.");
  }
}

}  // namespace kvaser_interface

RCLCPP_COMPONENTS_REGISTER_NODE(kvaser_interface::KvaserWriterNode)
