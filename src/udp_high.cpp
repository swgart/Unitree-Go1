#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>

// using UNITREE_LEGGED_SDK::UDP;
// using UNITREE_LEGGED_SDK::HighCmd;
// using UNITREE_LEGGED_SDK::HighState;
using namespace UNITREE_LEGGED_SDK;
class UDPHighBridge
{
public:
  UDP udp;

  HighCmd cmd = {0};
  HighState state = {0};

public:
  UDPHighBridge()
  : udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
  {
    udp.InitCmdData(cmd);
  }
};

class UDPHighNode : public rclcpp::Node
{
public:
  UDPHighNode()
  : Node("udp_high")
  {
    // timer_ = this->create_wall_timer(
    //   2ms,
    //   std::bind(&UDPHighNode::timer_callback, this));
    timer_ = create_wall_timer(
      static_cast<std::chrono::microseconds>(static_cast<int>(interval_ * 1000000.0)),
      std::bind(&UDPHighNode::timer_callback, this)
    );
    //Publishers
    pub_state_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 10);
    //Subscribers
    sub_cmd_ = this->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>(
      "high_cmd",
      10,
      std::bind(&UDPHighNode::cmd_callback, this, std::placeholders::_1)
    );
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pub_state_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_cmd_;
  

  double rate_, interval_;
  UDPHighBridge bridge_;
  ros2_unitree_legged_msgs::msg::HighState state_ros_;

  void cmd_callback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
  {
    //Convert ROS message to UDP command
    bridge_.cmd = rosMsg2Cmd(msg);
    //Send UDP command
    bridge_.udp.SetSend(bridge_.cmd);
    bridge_.udp.Send();
  }

  void timer_callback()
  {
    //send empty cmd to get state?
    bridge_.udp.Send();

    //Get state over UDP
    bridge_.udp.Recv();
    bridge_.udp.GetRecv(bridge_.state);
    RCLCPP_INFO_STREAM(get_logger(), bridge_.state.motorState[3].q);
    //Publish state as ROS message
    state_ros_ = state2rosMsg(bridge_.state);
    pub_state_->publish(state_ros_);
    // RCLCPP_INFO_STREAM(get_logger(), state_ros_.motor_state[1].q);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<UDPHighNode>());
  rclcpp::shutdown();
  return 0;
}
