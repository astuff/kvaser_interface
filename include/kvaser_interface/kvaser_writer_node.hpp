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

#ifndef KVASER_INTERFACE__KVASER_WRITER_NODE_HPP_
#define KVASER_INTERFACE__KVASER_WRITER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <can_msgs/msg/frame.hpp>
#include <can_fd_interface/msg/frame.hpp>
#include <memory>
#include <string>

#include "kvaser_interface/kvaser_interface.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace kvaser_interface
{

/// \brief KvaserWriterNode class which can pass messages to Kvaser hardware or virtual channels
class KvaserWriterNode final
  : public lc::LifecycleNode
{
public:
  /// \brief Default constructor
  explicit KvaserWriterNode(rclcpp::NodeOptions options);

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const lc::State & state) override;

  /// \brief Callback from transition to "activated" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const lc::State & state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const lc::State & state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const lc::State & state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const lc::State & state) override;

private:
  uint64_t hardware_id_;
  uint8_t circuit_id_;
  uint32_t bit_rate_;
  bool enable_echo_;
  std::shared_ptr<rclcpp::Subscription<can_msgs::msg::Frame>> frames_sub_;
  std::shared_ptr<rclcpp::Subscription<can_fd_interface::msg::Frame>> fd_frames_pub;

  KvaserCan can_writer_;
  void frame_callback(const can_msgs::msg::Frame::SharedPtr msg);
  void fd_frame_callback(const can_fd_interface::msg::Frame::SharedPtr msg);

  uint8_t tseg1_ = 0;
  uint8_t tseg2_ = 0;
  uint8_t sjw_ = 0;
  uint32_t data_bit_rate_ = 2000000;
  bool is_canfd_ = true;

};  // class KvaserWriterNode

}  // namespace kvaser_interface

#endif  // KVASER_INTERFACE__KVASER_WRITER_NODE_HPP_
