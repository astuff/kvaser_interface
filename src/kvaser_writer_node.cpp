// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "kvaser_interface/kvaser_writer_node.hpp"
#include "kvaser_interface/ros_utils.hpp"

#include <memory>
#include <string>

#include <lifecycle_msgs/msg/state.hpp>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

namespace AS
{
namespace CAN
{

KvaserWriterNode::KvaserWriterNode(rclcpp::NodeOptions options)
: lc::LifecycleNode("kvaser_writer_node", options)
{
  hardware_id_ = this->declare_parameter("hardware_id", 12345);
  circuit_id_ = this->declare_parameter("circuit_id", 0);
  bit_rate_ = this->declare_parameter("bit_rate", 500000);
  enable_echo_ = this->declare_parameter("enable_echo", false);
}

LNI::CallbackReturn KvaserWriterNode::on_configure(const lc::State & state)
{
  (void)state;
  ReturnStatuses ret;

  if ((ret = can_writer_.open(hardware_id_, circuit_id_, bit_rate_, enable_echo_)) != ReturnStatuses::OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CAN writer.");
    return LNI::CallbackReturn::FAILURE;
  }

  frames_sub_ = this->create_subscription<can_msgs::msg::Frame>("can_rx", 500,
    std::bind(&KvaserWriterNode::frame_callback, this, std::placeholders::_1));

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserWriterNode::on_cleanup(const lc::State & state)
{
  (void)state;
  frames_sub_.reset();
  return LNI::CallbackReturn::SUCCESS;
}

void KvaserWriterNode::frame_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    auto can_msg = KvaserRosUtils::from_ros_msg(*msg);
    can_writer_.write(std::move(can_msg));
  } else {
    RCLCPP_WARN(this->get_logger(), "Message received but node is not active.");
  }
}

}  // namespace CAN
}  // namespace AS

RCLCPP_COMPONENTS_REGISTER_NODE(AS::CAN::KvaserWriterNode)
