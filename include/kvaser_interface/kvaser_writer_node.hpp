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

#include "kvaser_interface/kvaser_interface.hpp"

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <can_msgs/msg/frame.hpp>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace AS
{
namespace CAN
{

/// \brief KvaserWriterNode class which can pass messages to Kvaser hardware or virtual channels
class KvaserWriterNode final
  : public lc::LifecycleNode
{
public:
  /// \brief Default constructor
  KvaserWriterNode(rclcpp::NodeOptions options);
  
  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const lc::State & state);

  /// \brief Callback from transition to "cleanup" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const lc::State & state);

private:
  uint8_t hardware_id_, circuit_id_;
  uint32_t bit_rate_;
  bool enable_echo_;
  std::shared_ptr<rclcpp::Subscription<can_msgs::msg::Frame>> frames_sub_;
  KvaserCan can_writer_;
  void frame_callback(const can_msgs::msg::Frame::SharedPtr msg);
};  // class KvaserWriterNode
}  // namespace CAN
}  // namespace AS

#endif  // KVASER_INTERFACE__KVASER_WRITER_NODE_HPP_