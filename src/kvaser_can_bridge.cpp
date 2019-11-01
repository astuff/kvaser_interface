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

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <memory>

#include "kvaser_interface/kvaser_reader_node.hpp"
#include "kvaser_interface/kvaser_writer_node.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  bool reader_started = false;
  bool writer_started = false;

  // ROS initialization
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto reader_node = std::make_shared<kvaser_interface::KvaserReaderNode>(options);
  auto writer_node = std::make_shared<kvaser_interface::KvaserWriterNode>(options);

  exec.add_node(reader_node->get_node_base_interface());
  exec.add_node(writer_node->get_node_base_interface());

  auto reader_configure_state = reader_node->configure();

  if (reader_configure_state.id() == State::PRIMARY_STATE_INACTIVE) {
    auto reader_activate_state =
      reader_node->activate();

    if (reader_activate_state.id() == State::PRIMARY_STATE_ACTIVE) {
      reader_started = true;
    }
  }

  auto writer_configure_state =
    writer_node->configure();

  if (writer_configure_state.id() == State::PRIMARY_STATE_INACTIVE) {
    auto writer_activate_state =
      writer_node->activate();

    if (writer_activate_state.id() == State::PRIMARY_STATE_ACTIVE) {
      writer_started = true;
    }
  }

  if (reader_started && writer_started) {
    exec.spin();
  }

  rclcpp::shutdown();

  return 0;
}
