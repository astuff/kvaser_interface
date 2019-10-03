// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "kvaser_interface/kvaser_reader_node.hpp"
#include "kvaser_interface/ros_utils.hpp"

#include <memory>
#include <string>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace AS
{
namespace CAN
{

KvaserReaderNode::KvaserReaderNode(rclcpp::NodeOptions options)
: lc::LifecycleNode("kvaser_interface_node", options)
{
  hardware_id_ = this->declare_parameter("hardware_id", 12345);
  circuit_id_ = this->declare_parameter("circuit_id", 0);
  bit_rate_ = this->declare_parameter("bit_rate", 500000);
  enable_echo_ = this->declare_parameter("enable_echo", false);
}

LNI::CallbackReturn KvaserReaderNode::on_configure(const lc::State & state)
{
  (void)state;
  ReturnStatuses ret;

  if ((ret = can_reader_.open(hardware_id_, circuit_id_, bit_rate_, enable_echo_)) != ReturnStatuses::OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CAN reader.");
    return LNI::CallbackReturn::FAILURE;
  }

  frames_pub_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 500);

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserReaderNode::on_activate(const lc::State & state)
{
  (void)state;
  frames_pub_->on_activate();
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserReaderNode::on_deactivate(const lc::State & state)
{
  (void)state;
  frames_pub_->on_deactivate();
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn KvaserReaderNode::on_cleanup(const lc::State & state)
{
  (void)state;
  frames_pub_.reset();
  return LNI::CallbackReturn::SUCCESS;
}

}  // namespace CAN
}  // namespace AS

RCLCPP_COMPONENTS_REGISTER_NODE(AS::CAN::KvaserReaderNode)
