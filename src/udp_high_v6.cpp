#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/msg/odometry.hpp"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : 
        // low_udp(LOWLEVEL),
        low_udp(LOWLEVEL),
        high_udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

Custom custom;

rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_cmd_vel;
rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pub_high;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

long cmd_vel_count = 0;

void cmdVelCallback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
{
    // printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    ros2_unitree_legged_msgs::msg::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom.high_state);

    // printf("position [X,Y,Z] = [%f, %f, %f]\n", high_state_ros.position[0],high_state_ros.position[1],high_state_ros.position[2]);
    
    pub_high->publish(high_state_ros);

    //publish odometry

    nav_msgs::msg::Odometry odom_ros;

    odom_ros.header.frame_id = "odom";
    odom_ros.child_frame_id = "base_link";

    odom_ros.pose.pose.position.x = custom.high_state.position[0];
    odom_ros.pose.pose.position.y = custom.high_state.position[1];
    odom_ros.pose.pose.position.z = custom.high_state.position[2];
    
    odom_ros.pose.pose.orientation.w = custom.high_state.imu.quaternion[0];
    odom_ros.pose.pose.orientation.x = custom.high_state.imu.quaternion[1];
    odom_ros.pose.pose.orientation.y = custom.high_state.imu.quaternion[2];
    odom_ros.pose.pose.orientation.z = custom.high_state.imu.quaternion[3];

    odom_ros.twist.twist.linear.x = custom.high_state.velocity[0];
    odom_ros.twist.twist.linear.y = custom.high_state.velocity[1];
    odom_ros.twist.twist.angular.z = custom.high_state.velocity[2];

    pub_odom->publish(odom_ros);

    // printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("udp_high");

    pub_high = node->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 1);
    pub_odom = node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    sub_cmd_vel = node->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1, cmdVelCallback);

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}