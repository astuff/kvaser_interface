// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "kvaser_interface/kvaser_interface_node.hpp"

#include <memory>
#include <string>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace AS
{
namespace CAN
{

KvaserInterfaceNode::KvaserInterfaceNode(rclcpp::NodeOptions options)
: lc::LifecycleNode("kvaser_interface_node", options)
{
  hardware_id_ = this->declare_parameter("hardware_id", 12345);
  circuit_id_ = this->declare_parameter("circuit_id", 0);
  bit_rate_ = this->declare_parameter("bit_rate", 500000);
  sub_frames_ = this->declare_parameter("sub_ros_frames", true);
  pub_frames_ = this->declare_parameter("pub_ros_frames", true);
  enable_echo_ = this->declare_parameter("enable_echo", false);
}

LNI::CallbackReturn KvaserInterfaceNode::on_configure(const lc::State & state)
{
  (void)state;
  ReturnStatuses ret;

  if ((ret = can_reader_.open(hardware_id_, circuit_id_, bit_rate_, enable_echo_)) != ReturnStatuses::OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CAN reader.");
    return LNI::CallbackReturn::FAILURE;
  }

  if ((ret = can_writer_.open(hardware_id_, circuit_id_, bit_rate_, enable_echo_)) != ReturnStatuses::OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CAN writer.");
    return LNI::CallbackReturn::FAILURE;
  }

  frames_pub_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 500);
  frames_sub_ = this->create_subscription<can_msgs::msg::Frame>("can_rx", 500,
    std::bind(&KvaserInterfaceNode::frame_callback, this, std::placeholders::_1));

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserInterfaceNode::on_activate(const lc::State & state)
{
  (void)state;
  return LNI::CallbackReturn::SUCCESS;
}

void KvaserInterfaceNode::frame_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (sub_frames_) {
    auto can_msg = KvaserRosUtils::from_ros_msg(*msg);
    can_writer_.write(std::move(*can_msg));
  }
}

std::unique_ptr<can_msgs::msg::Frame> KvaserRosUtils::to_ros_msg(const CanMsg & can_msg)
{
  auto ros_msg = std::unique_ptr<can_msgs::msg::Frame>(new can_msgs::msg::Frame);
  ros_msg->id = can_msg.id;
  ros_msg->dlc = can_msg.dlc;
  ros_msg->is_extended = can_msg.flags.ext_id;
  ros_msg->is_error = can_msg.flags.error_frame;
  ros_msg->is_rtr = can_msg.flags.rtr;

  if (can_msg.data.size() < 9) {
    std::copy(can_msg.data.begin(), can_msg.data.end(), ros_msg->data.begin());
  } else {
    std::copy(can_msg.data.begin(), can_msg.data.begin() + 8, ros_msg->data.begin());
  }

  return ros_msg;
}

std::unique_ptr<CanMsg> KvaserRosUtils::from_ros_msg(const can_msgs::msg::Frame & ros_msg)
{
  auto can_msg = std::unique_ptr<CanMsg>(new CanMsg);
  can_msg->id = ros_msg.id;
  can_msg->dlc = ros_msg.dlc;
  can_msg->flags.ext_id = ros_msg.is_extended;
  can_msg->flags.error_frame = ros_msg.is_error;
  can_msg->flags.rtr = ros_msg.is_rtr;

  auto msg_size = KvaserCanUtils::dlcToSize(ros_msg.dlc);

  for (size_t i = 0; i < msg_size; ++i) {
    can_msg->data.push_back(ros_msg.data[i]);
  }

  return can_msg;
}

}  // namespace CAN
}  // namespace AS

RCLCPP_COMPONENTS_REGISTER_NODE(AS::CAN::KvaserInterfaceNode)
