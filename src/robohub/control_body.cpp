#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "JAKAZuRobot.h"
#include <cassert>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "common.h"
using namespace std;
#define JK_PI (3.141592653589793)

class JAKAZuRobotNode : public rclcpp::Node
{
public:
    JAKAZuRobotNode()
        : Node("jakazu_body_node")
    {
        // Initialize JAKAZuRobot
        this->declare_parameter<std::string>("robot_ip", "172.30.95.93");
        auto robot_ip = this->get_parameter("robot_ip").as_string();
        robot_ = std::make_unique<JAKAZuRobot>();
        int ret = robot_->login_in(robot_ip.c_str());
        ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");
        robot_->ext_enable_off();

        robot_->ext_power_off();

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        robot_->ext_power_on();

        std::this_thread::sleep_for(std::chrono::milliseconds(20000));
        robot_->ext_enable_on();
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        robot_->ext_jog_to(1, 0, 10, 10);
        robot_->ext_jog_to(2, 0, 10, 10);
        robot_->ext_jog_to(3, 0, 10, 10);
        robot_->ext_jog_to(4, 0, 10, 10);
        RCLCPP_INFO(this->get_logger(), "The initiation is done.");
        sub_joint1_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "head_up_down", 10, std::bind(&JAKAZuRobotNode::joint1Callback, this, std::placeholders::_1));
        sub_joint2_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "head_left_right", 10, std::bind(&JAKAZuRobotNode::joint2Callback, this, std::placeholders::_1));
        sub_joint3_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "waist_up_down", 10, std::bind(&JAKAZuRobotNode::joint3Callback, this, std::placeholders::_1));
        sub_joint4_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "waist_left_right", 10, std::bind(&JAKAZuRobotNode::joint4Callback, this, std::placeholders::_1));
    }

    ~JAKAZuRobotNode()
    {
        robot_->login_out();
    }

private:
    void joint1Callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto current_time = this->get_clock()->now();
        msg->header.stamp = current_time;
        set_head_max_min(msg->position[0]);
        // cout << msg->position[0] << endl;
        robot_->ext_jog_to(2, msg->position[0], 10, 10);
        RCLCPP_INFO(this->get_logger(), "Current head up down position: %f", msg->position[0]);
    }

    void joint2Callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        set_head_max_min(msg->position[0]);
        // cout << msg->position[0] << endl;
        robot_->ext_jog_to(1, msg->position[0], 10, 10);
        RCLCPP_INFO(this->get_logger(), "Current head left right position: %f", msg->position[0]);
    }

    void joint3Callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        set_max_min(msg->position[0]);
        // cout << msg->position[0] << endl;
        robot_->ext_jog_to(4, msg->position[0], 10, 10);
        RCLCPP_INFO(this->get_logger(), "Current waist up down position: %f", msg->position[0]);
    }

    void joint4Callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        set_max_min(msg->position[0]);
        // cout << msg->position[0] << endl;
        robot_->ext_jog_to(3, msg->position[0], 10, 10);
        RCLCPP_INFO(this->get_logger(), "Current waist left right position: %f", msg->position[0]);
    }
    void set_max_min(double &value)
    {
        if (value > 3)
        {
            value = 3;
        }
        if (value < -3)
        {
            value = -3;
        }
    }
    void set_head_max_min(double &value)
    {
        if (value > 10)
        {
            value = 10;
        }
        if (value < -10)
        {
            value = -10;
        }
    }
    std::unique_ptr<JAKAZuRobot> robot_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint1_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint2_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint3_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint4_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JAKAZuRobotNode>());
    rclcpp::shutdown();
    return 0;
}