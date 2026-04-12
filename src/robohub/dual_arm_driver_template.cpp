#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "k1_msgs/srv/joint_move.hpp"
#include "k1_msgs/srv/linear_move.hpp"
#include "k1_msgs/srv/kine_inverse7.hpp"

#include "JAKAZuRobot.h"
#include <cassert>
#include <chrono>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <thread>
#include "common.h"
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>

#define JK_PI (3.141592653589793)

using std::placeholders::_1;
using std::placeholders::_2;

class JakaRobotDriver : public rclcpp::Node
{
public:
    JakaRobotDriver() : Node("jaka_robot_driver")
    {
        // Create publishers
        left_arm_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("left_arm/joint_states", 10);
        left_arm_joint_state_vel_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("left_arm/joint_states_vel", 10);
        left_arm_tcp_pose_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("left_arm/cur_tcp_pose", 10);
        // left_arm_gripper_val_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("left_arm/cur_gripper_val", 10);
        left_arm_raw_torque_sensor_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("left_arm/raw_torque_sensor_val", 10);
        left_arm_command_servo_j_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("left_arm/robot_command_servo_j", 10);

        right_arm_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("right_arm/joint_states", 10);
        right_arm_joint_state_vel_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("right_arm/joint_states_vel", 10);
        right_arm_tcp_pose_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("right_arm/cur_tcp_pose", 10);
        // right_arm_gripper_val_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("right_arm/cur_gripper_val", 10);
        right_arm_raw_torque_sensor_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("right_arm/raw_torque_sensor_val", 10);
        right_arm_command_servo_j_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("right_arm/robot_command_servo_j", 10);

        dual_arm_command_p_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("robot_command_servo_p", 10);
        dual_arm_command_j_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("robot_command_servo_j", 10);

        // Note: the timer will be created after entering servo mode
        // dual_arm_command_p_timer_ = this->create_wall_timer(std::chrono::milliseconds(4), std::bind(&JakaRobotDriver::dual_arm_command_p_send_callback, this));
        dual_arm_command_j_timer_ = this->create_wall_timer(std::chrono::milliseconds(8), std::bind(&JakaRobotDriver::dual_arm_command_j_send_callback, this));
        // Create services
        joint_move_service_ = this->create_service<k1_msgs::srv::JointMove>("joint_move", std::bind(&JakaRobotDriver::joint_move_callback, this, _1, _2));
        linear_move_service_ = this->create_service<k1_msgs::srv::LinearMove>("linear_move", std::bind(&JakaRobotDriver::linear_move_callback, this, _1, _2));
        kine_inverse_service_ = this->create_service<k1_msgs::srv::KineInverse7>("kine_inverse7", std::bind(&JakaRobotDriver::kine_inverse_callback, this, _1, _2));
        force_control_mode_enable_service_ = this->create_service<std_srvs::srv::SetBool>("force_control_mode_enable",
                                                                                          std::bind(&JakaRobotDriver::force_control_mode_enable_callback, this, _1, _2));
        // TODO: temporarily reuse the left-arm signal
        servo_move_enable_service_ = this->create_service<std_srvs::srv::SetBool>("left_arm/servo_move_enable",
                                                                                  // servo_move_enable_service_ = this->create_service<std_srvs::srv::setbool>("servo_move_enable",
                                                                                  std::bind(&JakaRobotDriver::servo_move_enable_callback, this, _1, _2));

        // Subscribe to topics
        servo_j_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("robot_command_servo_j", 10, std::bind(&JakaRobotDriver::servo_j7_callback, this, _1));
        dual_arm_servo_p_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("robot_command_servo_p", 10, std::bind(&JakaRobotDriver::servo_p_callback, this, _1));
        // Subscribe to the interpolated topic (1000 Hz) instead of the raw topic (90 Hz)
        left_arm_servo_p_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("left_arm/interpolated_robot_command", 10, std::bind(&JakaRobotDriver::servo_p_callback_left_arm, this, _1));
        right_arm_servo_p_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("right_arm/interpolated_robot_command", 10, std::bind(&JakaRobotDriver::servo_p_callback_right_arm, this, _1));
        left_arm_servo_j_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("left_arm/robot_command_servo_j", 10, std::bind(&JakaRobotDriver::servo_j_callback_left_arm, this, _1));
        right_arm_servo_j_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("right_arm/robot_command_servo_j", 10, std::bind(&JakaRobotDriver::servo_j_callback_right_arm, this, _1));
        // Initialize the robot
        dual_command_servo_p_val.resize(12);
        dual_command_servo_j_val.resize(14);
        robot_ = std::make_shared<JAKAZuRobot>();
        // Declare parameters without forcing defaults so launch-file values take priority
        if (!this->has_parameter("robot_ip")) {
            this->declare_parameter<std::string>("robot_ip", "192.168.2.200");
        }
        robot_ip = this->get_parameter("robot_ip").as_string();
        if (!this->has_parameter("robot_running_mode")) {
            this->declare_parameter<std::string>("robot_running_mode", "teleop");
        }
        robot_running_mode = this->get_parameter("robot_running_mode").as_string();
        if (!this->has_parameter("verbose_mode")) {
            this->declare_parameter<bool>("verbose_mode", false);
        }
        verbose_mode = this->get_parameter("verbose_mode").as_bool();
        msg_pre_left_joint_state.resize(7);
        msg_pre_right_joint_state.resize(7);

        // this->declare_parameter<int>("robot_arm_index", 0); // 0 for left arm, 1 for right arm
        // robot_arm_index = this->get_parameter("robot_arm_index").as_int();

        RCLCPP_INFO(this->get_logger(), "Attempting to connect to robot at IP: %s", robot_ip.c_str());
        int login_ret = robot_->login_in(robot_ip.c_str());
        if (login_ret != ERR_SUCC)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to login to robot at %s, error code: %d", robot_ip.c_str(), login_ret);
            throw std::runtime_error("Failed to login to robot");
        }
        RCLCPP_INFO(this->get_logger(), "Successfully logged in to robot");

        // Initialize EDG support (required by the new SDK)
        // Parameters: edg_init(BOOL en, const char* edg_stat_ip, int edg_port, int edg_mode)
        // edg_stat_ip: client IP used by the SDK to receive EDG feedback - must be the local host IP
        RCLCPP_INFO(this->get_logger(), "Initializing EDG functionality...");
        // Declare parameters while still allowing launch-file values to override them
        if (!this->has_parameter("edg_local_ip")) {
            this->declare_parameter<std::string>("edg_local_ip", "192.168.2.101");
        }
        if (!this->has_parameter("edg_port")) {
            this->declare_parameter<int>("edg_port", 10011);
        }
        std::string local_ip = this->get_parameter("edg_local_ip").as_string();
        int edg_port = this->get_parameter("edg_port").as_int();
        int edg_mode = 0;  // 0: all EDG-related interfaces are available
        int edg_ret = robot_->edg_init(true, local_ip.c_str(), edg_port, edg_mode);
        if (edg_ret != ERR_SUCC)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize EDG, error code: %d", edg_ret);
            throw std::runtime_error("Failed to initialize EDG");
        }
        RCLCPP_INFO(this->get_logger(), "EDG initialized successfully with IP: %s, Port: %d", local_ip.c_str(), edg_port);

        robot_->clear_error();
        // robot_->power_off();
        // std::this_thread::sleep_for(std::chrono::seconds(3));

        RCLCPP_INFO(this->get_logger(), "Powering on robot...");
        int power_ret = robot_->power_on();
        if (power_ret != ERR_SUCC)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to power on robot, error code: %d", power_ret);
            throw std::runtime_error("Failed to power on robot");
        }
        
        RCLCPP_INFO(this->get_logger(), "Enabling robot...");
        int enable_ret = robot_->enable_robot();
        if (enable_ret != ERR_SUCC)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to enable robot, error code: %d", enable_ret);
            throw std::runtime_error("Failed to enable robot");
        }
        RCLCPP_INFO(this->get_logger(), "Robot powered on and enabled successfully");

        // Joint names

        // // ************************ test force control mode to optimize dual arm operation
        // // New SDK API: robot_zero_ftsensor(int sensor_id) - only the sensor ID is required
        // robot_->robot_zero_ftsensor(0);
        // robot_->robot_zero_ftsensor(1);
        // std::this_thread::sleep_for(std::chrono::seconds(1));
        // robot_->robot_set_cst_ftframe(LEFT, 1);
        // robot_->robot_set_cst_ftconfig(LEFT, 0, 1, 10, 1, 0);
        // robot_->robot_set_cst_ftconfig(LEFT, 1, 1, 10, 1, 0);
        // robot_->robot_set_cst_ftconfig(LEFT, 2, 0, 500, 1, 0);
        // robot_->robot_set_cst_ftconfig(LEFT, 3, 0, 10, 1, 0);
        // robot_->robot_set_cst_ftconfig(LEFT, 4, 0, 10, 1, 0);
        // robot_->robot_set_cst_ftconfig(LEFT, 5, 0, 10, 1, 0);
        // robot_->robot_enable_force_control(LEFT);
        // // ************************ end test force control mode to optimize dual arm operation

        // Wait until EDG data is ready
        RCLCPP_INFO(this->get_logger(), "Waiting for EDG data to be ready...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Create timers and start periodic data acquisition
        RCLCPP_INFO(this->get_logger(), "Creating timers for data acquisition...");
        left_arm_timer_ = this->create_wall_timer(std::chrono::milliseconds(8), std::bind(&JakaRobotDriver::left_arm_timer_callback, this));
        right_arm_timer_ = this->create_wall_timer(std::chrono::milliseconds(8), std::bind(&JakaRobotDriver::right_arm_timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "The robot initiation is done. Ready to receive servo commands.");
    }

    ~JakaRobotDriver()
    {
        robot_->robot_disable_force_control(LEFT);
        robot_->login_out();
    }

private:
    std::shared_ptr<JAKAZuRobot> robot_;
    std::string robot_ip;
    std::string robot_running_mode;
    bool verbose_mode;
    // int robot_arm_index;
    // sensor_msgs::msg::JointState::SharedPtr dual_arm_command_p_msg_;

    // sensor_msgs::msg::JointState msg_pre_left_joint_state;
    // sensor_msgs::msg::JointState msg_pre_right_joint_state;
    std::vector<double> msg_pre_left_joint_state;
    std::vector<double> msg_pre_right_joint_state;
    // double loop_time = 0.008; // 125 Hz transfer rate, 0.008 s period
    double loop_time = 0.011; // 90 Hz transfer rate, 0.011 s period

    std::vector<double> dual_command_servo_p_val;
    std::vector<double> dual_command_servo_j_val;
    bool servo_move_enabled_ = false;
    bool left_arm_command_p_coming = false;
    bool right_arm_command_p_coming = false;
    bool left_arm_command_j_coming = false;
    bool right_arm_command_j_coming = false;
    // Joint names
    std::string joint_names_[7] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};

    // Kinematics-related variables
    int full_dh;
    int num_joint_kin;

    // Class members used to store real-time joint values
    JointValue left_arm_joint_values;
    JointValue right_arm_joint_values;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_joint_state_vel_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_tcp_pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_gripper_val_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_raw_torque_sensor_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_command_servo_j_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_joint_state_vel_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_tcp_pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_gripper_val_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_raw_torque_sensor_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_command_servo_j_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr dual_arm_command_p_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr dual_arm_command_j_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_j_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr dual_arm_servo_p_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_arm_servo_p_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_arm_servo_p_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_arm_servo_j_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_arm_servo_j_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_gripper_val_;

    rclcpp::Service<k1_msgs::srv::JointMove>::SharedPtr joint_move_service_;
    rclcpp::Service<k1_msgs::srv::LinearMove>::SharedPtr linear_move_service_;
    rclcpp::Service<k1_msgs::srv::KineInverse7>::SharedPtr kine_inverse_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr servo_move_enable_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr force_control_mode_enable_service_;

    rclcpp::TimerBase::SharedPtr left_arm_timer_;
    rclcpp::TimerBase::SharedPtr right_arm_timer_;
    rclcpp::TimerBase::SharedPtr dual_arm_command_p_timer_;
    rclcpp::TimerBase::SharedPtr dual_arm_command_j_timer_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint1_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint2_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint3_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint4_;
    // void joint1Callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    // {
    //     auto current_time = this->get_clock()->now();
    //     msg->header.stamp = current_time;
    //     set_head_max_min(msg->position[0]);
    //     // cout << msg->position[0] << endl;
    //     robot_->ext_jog_to(2, msg->position[0], 10, 10);
    //     RCLCPP_INFO(this->get_logger(), "Current head up down position: %f", msg->position[0]);
    // }

    // void joint2Callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    // {
    //     set_head_max_min(msg->position[0]);
    //     // cout << msg->position[0] << endl;
    //     robot_->ext_jog_to(1, msg->position[0], 10, 10);
    //     RCLCPP_INFO(this->get_logger(), "Current head left right position: %f", msg->position[0]);
    // }

    // void joint3Callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    // {
    //     set_max_min(msg->position[0]);
    //     // cout << msg->position[0] << endl;
    //     robot_->ext_jog_to(4, msg->position[0], 10, 10);
    //     RCLCPP_INFO(this->get_logger(), "Current waist up down position: %f", msg->position[0]);
    // }

    // void joint4Callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    // {
    //     set_max_min(msg->position[0]);
    //     // cout << msg->position[0] << endl;
    //     robot_->ext_jog_to(3, msg->position[0], 10, 10);
    //     RCLCPP_INFO(this->get_logger(), "Current waist left right position: %f", msg->position[0]);
    // }
    // void set_max_min(double &value)
    // {
    //     if (value > 3)
    //     {
    //         value = 3;
    //     }
    //     if (value < -3)
    //     {
    //         value = -3;
    //     }
    // }
    // void set_head_max_min(double &value)
    // {
    //     if (value > 10)
    //     {
    //         value = 10;
    //     }
    //     if (value < -10)
    //     {
    //         value = -10;
    //     }
    // }
    void dual_arm_command_p_send_callback()
    {
        if (left_arm_command_p_coming && right_arm_command_p_coming && dual_command_servo_p_val.size() == 12)
        {

            RCLCPP_INFO(this->get_logger(), "before send dual command p ");
            sensor_msgs::msg::JointState dual_command_servo_p_msg;
            auto current_time = this->get_clock()->now();
            dual_command_servo_p_msg.header.stamp = current_time;
            dual_command_servo_p_msg.position.resize(12);
            dual_command_servo_p_msg.position = dual_command_servo_p_val;
            dual_arm_command_p_publisher_->publish(dual_command_servo_p_msg);
            RCLCPP_INFO(this->get_logger(), "after send dual command p ");
            left_arm_command_p_coming = false;
            right_arm_command_p_coming = false;
        }
        else
        {
            //    RCLCPP_INFO(this->get_logger(), "wait for command p sig");
        }
    }

    void dual_arm_command_j_send_callback()
    {
        if (verbose_mode)
        {
            printf("receive j\n");
            printf("receive left %d\n", left_arm_command_j_coming);
            printf("receive right %d\n", right_arm_command_j_coming);
            printf("receive dual  %d\n", dual_command_servo_j_val.size());
            printf("receive   %s \n", robot_running_mode.c_str());
            std::cout << robot_running_mode << std::endl;
        }

        if (left_arm_command_j_coming && right_arm_command_j_coming && dual_command_servo_j_val.size() == 14 && robot_running_mode == "act_exe")
        {

            RCLCPP_INFO(this->get_logger(), "before send dual command j ");
            sensor_msgs::msg::JointState dual_command_servo_j_msg;
            auto current_time = this->get_clock()->now();
            dual_command_servo_j_msg.header.stamp = current_time;
            dual_command_servo_j_msg.position.resize(14);
            dual_command_servo_j_msg.position = dual_command_servo_j_val;
            dual_arm_command_j_publisher_->publish(dual_command_servo_j_msg);
            RCLCPP_INFO(this->get_logger(), "after send dual command j ");
            left_arm_command_j_coming = false;
            right_arm_command_j_coming = false;
        }
        else
        {
            //    RCLCPP_INFO(this->get_logger(), "wait for command p sig");
        }
    }

    void left_arm_timer_callback()
    {
        sensor_msgs::msg::JointState msg_joint_state;
        sensor_msgs::msg::JointState msg_tcp_pose;
        sensor_msgs::msg::JointState msg_gripper;
        sensor_msgs::msg::JointState msg_raw_torque;
        sensor_msgs::msg::JointState msg_joint_state_vel;

        auto current_time = this->get_clock()->now();
        msg_joint_state.header.stamp = current_time;
        msg_tcp_pose.header.stamp = current_time;
        msg_gripper.header.stamp = current_time;
        msg_raw_torque.header.stamp = current_time;
        msg_joint_state_vel.header.stamp = current_time;

        msg_joint_state.name.assign(joint_names_, joint_names_ + 7);
        msg_gripper.name.assign(joint_names_, joint_names_ + 7);
        msg_raw_torque.name.assign(joint_names_, joint_names_ + 6);
        msg_joint_state_vel.name.assign(joint_names_, joint_names_ + 7);

        CartesianPose cartesian_pose;
        // Note: the new SDK no longer requires edg_recv; edg_init receives data automatically
        // be careful! cartesian read rpy in degrees, but requires radius to control servo p
        int ret = robot_->edg_get_stat(0, &left_arm_joint_values, &cartesian_pose);
        
        if (ret == ERR_SUCC) // 0 for left arm
        {
            
            // joint vals
            msg_joint_state.position.assign(left_arm_joint_values.jVal, left_arm_joint_values.jVal + 7);
            tf2::Quaternion quaternion;
            quaternion.setRPY(cartesian_pose.rpy.rx / 180 * M_PI, cartesian_pose.rpy.ry / 180 * M_PI, cartesian_pose.rpy.rz / 180 * M_PI);

            msg_tcp_pose.position.resize(7);
            msg_tcp_pose.position[0] = cartesian_pose.tran.x;
            msg_tcp_pose.position[1] = cartesian_pose.tran.y;
            msg_tcp_pose.position[2] = cartesian_pose.tran.z;
            msg_tcp_pose.position[3] = quaternion.w();
            msg_tcp_pose.position[4] = quaternion.x();
            msg_tcp_pose.position[5] = quaternion.y();
            msg_tcp_pose.position[6] = quaternion.z();
        }
        else
        {
            printf("[LEFT ARM] edg_get_stat failed with error code: %d\n", ret);
            // TCP Pose
            // msg_tcp_pose.pose.position.x = cartesian_pose.tran.x;
            // msg_tcp_pose.pose.position.y = cartesian_pose.tran.y;
            // msg_tcp_pose.pose.position.z = cartesian_pose.tran.z;
            // // Convert RPY to quaternion
            // msg_tcp_pose.pose.orientation.w = quaternion.w();
            // msg_tcp_pose.pose.orientation.x = quaternion.x();
            // msg_tcp_pose.pose.orientation.y = quaternion.y();
            // msg_tcp_pose.pose.orientation.z = quaternion.z();
        }

        if (msg_pre_left_joint_state.size() != 0)
        {
            for (int i = 0; i < 7; i++)
            {
                msg_joint_state_vel.velocity.push_back((msg_joint_state.position[i] - msg_pre_left_joint_state[i]) / loop_time);
            }
            left_arm_joint_state_vel_publisher_->publish(msg_joint_state_vel);
        }

        for (int i = 0; i < 7; i++)
        {
            msg_pre_left_joint_state[i] = msg_joint_state.position[i];
        }

        left_arm_joint_state_publisher_->publish(msg_joint_state);
        left_arm_tcp_pose_publisher_->publish(msg_tcp_pose);

        // // check_fct_state(robot);

        // New SDK API: robot_get_ftsensor_stat(int sensor_id, int* status, int* errcode, double* ft_original, double* ft_actual)
        // sensor_id: sensor ID (1 for the left arm)
        int status = -3;
        int errorcode = -1;
        double ft_original[6] = {};
        double ft_actual[6] = {};
        int ft_ret = robot_->robot_get_ftsensor_stat(1, &status, &errorcode, ft_original, ft_actual);
        if (ft_ret != 0)
        {
            printf("robot_get_ftsensor_stat failed!\n");
        }
        else
        {
            msg_raw_torque.position.assign(ft_original, ft_original + 6);
            left_arm_raw_torque_sensor_publisher_->publish(msg_raw_torque);
        }
    }

    void right_arm_timer_callback()
    {
        sensor_msgs::msg::JointState msg_joint_state;
        // geometry_msgs::msg::PoseStamped msg_tcp_pose;
        sensor_msgs::msg::JointState msg_tcp_pose;
        sensor_msgs::msg::JointState msg_gripper;
        sensor_msgs::msg::JointState msg_raw_torque;
        sensor_msgs::msg::JointState msg_joint_state_vel;

        auto current_time = this->get_clock()->now();
        msg_joint_state.header.stamp = current_time;
        msg_tcp_pose.header.stamp = current_time;
        msg_gripper.header.stamp = current_time;
        msg_raw_torque.header.stamp = current_time;
        msg_joint_state_vel.header.stamp = current_time;

        msg_joint_state.name.assign(joint_names_, joint_names_ + 7);
        msg_gripper.name.assign(joint_names_, joint_names_ + 7);
        msg_raw_torque.name.assign(joint_names_, joint_names_ + 6);
        msg_joint_state_vel.name.assign(joint_names_, joint_names_ + 7);

        CartesianPose cartesian_pose;
        // Note: the new SDK no longer requires edg_recv; edg_init receives data automatically
        int ret = robot_->edg_get_stat(1, &right_arm_joint_values, &cartesian_pose);
        
        if (ret == ERR_SUCC) // 1 for right arm
        {
            
            // joint vals
            msg_joint_state.position.assign(right_arm_joint_values.jVal, right_arm_joint_values.jVal + 7);

            msg_tcp_pose.position.resize(7);
            // TCP Pose
            msg_tcp_pose.position[0] = cartesian_pose.tran.x;
            msg_tcp_pose.position[1] = cartesian_pose.tran.y;
            msg_tcp_pose.position[2] = cartesian_pose.tran.z;
            // Convert RPY to quaternion
            tf2::Quaternion quaternion;
            quaternion.setRPY(cartesian_pose.rpy.rx / 180 * M_PI, cartesian_pose.rpy.ry / 180 * M_PI, cartesian_pose.rpy.rz / 180 * M_PI);
            msg_tcp_pose.position[3] = quaternion.w();
            msg_tcp_pose.position[4] = quaternion.x();
            msg_tcp_pose.position[5] = quaternion.y();
            msg_tcp_pose.position[6] = quaternion.z();
        }
        else
        {
            printf("[RIGHT ARM] edg_get_stat failed with error code: %d\n", ret);
        }

        if (msg_pre_right_joint_state.size() != 0)
        {
            for (int i = 0; i < 7; i++)
            {
                msg_joint_state_vel.velocity.push_back((msg_joint_state.position[i] - msg_pre_right_joint_state[i]) / loop_time);
            }
            right_arm_joint_state_vel_publisher_->publish(msg_joint_state_vel);
        }

        for (int i = 0; i < 7; i++)
        {
            msg_pre_right_joint_state[i] = msg_joint_state.position[i];
        }

        right_arm_joint_state_publisher_->publish(msg_joint_state);
        right_arm_tcp_pose_publisher_->publish(msg_tcp_pose);

        // New SDK API: robot_get_ftsensor_stat(int sensor_id, int* status, int* errcode, double* ft_original, double* ft_actual)
        // sensor_id: sensor ID (2 for the right arm)
        int status = -3;
        int errorcode = -1;
        double ft_original[6] = {};
        double ft_actual[6] = {};
        int ft_ret = robot_->robot_get_ftsensor_stat(2, &status, &errorcode, ft_original, ft_actual);

        if (ft_ret != 0)
        {
            printf("robot_get_ftsensor_stat failed!\n");
        }
        else
        {
            msg_raw_torque.position.assign(ft_original, ft_original + 6);
            right_arm_raw_torque_sensor_publisher_->publish(msg_raw_torque);
        }
    }

    void linear_move_callback(const std::shared_ptr<k1_msgs::srv::LinearMove::Request> request,
                              std::shared_ptr<k1_msgs::srv::LinearMove::Response> response)
    {
        CartesianPose target_pos[2] = {
            {request->end_position[0], request->end_position[1], request->end_position[2], 0, 0, 0},
            {request->end_position[0], request->end_position[1], request->end_position[2], 0, 0, 0}};
        MoveMode moveop[2];
        if (request->move_mode == 0)
        {
            moveop[0] = ABS;
            moveop[1] = ABS;
        }
        else
        {
            moveop[0] = INCR;
            moveop[1] = INCR;
        }
        double vel[2] = {1, 1};
        double acc[2] = {1, 1};

        if (robot_->robot_run_multi_movl(-1, moveop, request->is_blocking, target_pos, vel, acc) == ERR_SUCC)
        {
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }

    void joint_move_callback(const std::shared_ptr<k1_msgs::srv::JointMove::Request> request,
                             std::shared_ptr<k1_msgs::srv::JointMove::Response> response)
    {
        JointValue joint_values[2]{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        RCLCPP_INFO(this->get_logger(), "receive joint move");
        switch (request->robot_arm_index)
        {
        case -1:
            for (int i = 0; i < 7; i++)
            {
                joint_values[0].jVal[i] = request->joint_positions[i];
                RCLCPP_INFO(this->get_logger(), "Left arm joint %d value: %f", i, joint_values[0].jVal[i]);
                joint_values[1].jVal[i] = request->joint_positions[i + 7];
                RCLCPP_INFO(this->get_logger(), "Right arm joint %d value: %f", i, joint_values[1].jVal[i]);
            }
            /* code */
            break;
        case 0:
            for (int i = 0; i < 7; i++)
            {
                joint_values[0].jVal[i] = request->joint_positions[i];
                RCLCPP_INFO(this->get_logger(), "Left arm joint %d value: %f", i, joint_values[0].jVal[i]);
            }
            /* code */
            break;
        case 1:
            for (int i = 0; i < 7; i++)
            {
                joint_values[1].jVal[i] = request->joint_positions[i];
                RCLCPP_INFO(this->get_logger(), "Right arm joint %d value: %f", i, joint_values[1].jVal[i]);
            }
            /* code */
            break;

        default:
            break;
        }
        MoveMode moveop[2];
        if (request->move_mode == 0)
        {
            moveop[0] = ABS;
            moveop[1] = ABS;
        }
        else
        {
            moveop[0] = INCR;
            moveop[1] = INCR;
        }
        double vel[2] = {1, 1};
        double acc[2] = {1, 1};
        double tol[2] = {0, 0};
        double id[2] = {0, 0};
        if (robot_->robot_run_multi_movj(request->robot_arm_index, moveop, request->is_blocking, joint_values, vel, acc) == ERR_SUCC)
        {
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }

    // void linear_move_callback(const std::shared_ptr<k1_msgs::srv::LinearMove::Request> request,
    //                           std::shared_ptr<k1_msgs::srv::LinearMove::Response> response) {
    //     CartesianPose pose;
    //     pose.tran.x = request->end_position.position.x;
    //     pose.tran.y = request->end_position.position.y;
    //     pose.tran.z = request->end_position.position.z;

    //     if (robot_->robot_run_multi_movl(0, ABS, true, &pose, nullptr, nullptr) == ERR_SUCC) {
    //         response->success = true;
    //     } else {
    //         response->success = false;
    //     }
    // }
    void kine_inverse_callback(k1_msgs::srv::KineInverse7::Request::SharedPtr request,
                               k1_msgs::srv::KineInverse7::Response::SharedPtr response)
    {
        try
        {
            JointValue ref_joint_values, joint_values;
            std::copy(request->ref_joint_pos.begin(), request->ref_joint_pos.end(), ref_joint_values.jVal);
            CartesianPose cartesian_pose;
            cartesian_pose.tran.x = request->cartesian_pose[0];
            cartesian_pose.tran.y = request->cartesian_pose[1];
            cartesian_pose.tran.z = request->cartesian_pose[2];
            cartesian_pose.rpy.rx = request->cartesian_pose[3];
            cartesian_pose.rpy.ry = request->cartesian_pose[4];
            cartesian_pose.rpy.rz = request->cartesian_pose[5];
            auto result = robot_->kine_inverse(request->robot_arm_index, &ref_joint_values, &cartesian_pose, &joint_values);
            response->success = (result == 0);
            response->message = response->success ? "Success" : "Error: " + std::to_string(result);
            if (response->success)
            {
                std::vector<double> computed_joint_values;
                // std::copy(joint_values.jVal, joint_values.jVal+7, computed_joint_values);
                response->joint_positions.assign(std::begin(joint_values.jVal), std::end(joint_values.jVal));
            }
            else
            {
                response->joint_positions.clear();
            }
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message = e.what();
        }
    }
    void force_control_mode_enable_callback(std_srvs::srv::SetBool::Request::SharedPtr request,
                                            std_srvs::srv::SetBool::Response::SharedPtr response)
    {
        // ************************ test force control mode to optimize dual arm operation
        // New SDK API: robot_zero_ftsensor(int sensor_id) - only the sensor ID is required
        // Zero the left-arm sensors (sensor IDs 0 and 1)
        robot_->robot_zero_ftsensor(1);
        robot_->robot_zero_ftsensor(2);
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
        // **********Force-compliance features are not needed here, so they remain commented out**********//
        // robot_->robot_set_cst_ftframe(LEFT, 1);
        // robot_->robot_set_cst_ftconfig(LEFT, 0, 1, 10, 1, 0);
        // robot_->robot_set_cst_ftconfig(LEFT, 1, 1, 10, 1, 0);
        // robot_->robot_set_cst_ftconfig(LEFT, 2, 0, 500, 1, 0);
        // robot_->robot_set_cst_ftconfig(LEFT, 3, 0, 10, 1, 0);
        // robot_->robot_set_cst_ftconfig(LEFT, 4, 0, 10, 1, 0);
        // robot_->robot_set_cst_ftconfig(LEFT, 5, 0, 10, 1, 0);
        // robot_->robot_enable_force_control(LEFT);
        // **********Force-compliance features are not needed here, so they remain commented out**********//
        // ************************ end test force control mode to optimize dual arm operation
    }

    void servo_move_enable_callback(std_srvs::srv::SetBool::Request::SharedPtr request,
                                    std_srvs::srv::SetBool::Response::SharedPtr response)
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "change servo mode");
            servo_move_enabled_ = request->data;
            if (servo_move_enabled_)
            {

                robot_->servo_move_use_joint_LPF(0.5);
                // // robot_->servo_move_use_joint_LPF(0.5);
                // if (robot_running_mode == "act_exe")
                // {
                //     robot_->servo_move_use_joint_NLF(60, 60, 60);
                // }
                // else if (robot_running_mode == "teleop")
                // {
                //     // robot_->servo_move_use_carte_NLF(250, 250, 250, 50, 50, 50);
                //     robot_->servo_move_use_joint_LPF(0.5);
                //     // robot_->servo_move_use_carte_NLF(5000, 2500, 2500, 500, 250, 250);
                //     // robot_->servo_move_use_carte_NLF(5000, 5000, 5000, 2000, 2000, 2000);
                // }
                // else if (robot_running_mode == "test")
                // {
                //     robot_->servo_move_use_none_filter();
                // }
                // else
                // {
                //     robot_->servo_move_use_carte_NLF(5000, 2500, 2500, 500, 250, 250);
                // }
            }
            // New SDK API: servo_move_enable(BOOL enable, BOOL is_block, int robindex)
            // -1 controls both arms together, and true means blocking mode
            int result = robot_->servo_move_enable(servo_move_enabled_, true, -1);
            response->success = (result == 0);
            response->message = servo_move_enabled_ ? "Enabled SERVO MOVE mode" : "Disabled SERVO MOVE mode";
            if (!response->success)
            {
                response->message += " (Error code: " + std::to_string(result) + ")";
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), " to %d", servo_move_enabled_);
            }
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message = e.what();
        }
    }

    void servo_p_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "receive servo p");
        if (!servo_move_enabled_)
        {
            RCLCPP_WARN(this->get_logger(), "Servo Move is not enabled. Please enable it first.");
            return;
        }

        try
        {
            std::vector<double> joint_positions = msg->position;
            auto move_mode = msg->name.empty() ? ABS : INCR;

            if (joint_positions.size() != 12)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid number of joint positions: %zu", joint_positions.size());
                return;
            }

            CartesianPose pose_left;
            pose_left.tran.x = joint_positions[0];
            pose_left.tran.y = joint_positions[1];
            pose_left.tran.z = joint_positions[2];
            pose_left.rpy.rx = joint_positions[3];
            pose_left.rpy.ry = joint_positions[4];
            pose_left.rpy.rz = joint_positions[5];
            RCLCPP_INFO(this->get_logger(), "servo p pose_left: %f, %f, %f, %f, %f, %f",
                        joint_positions[0], joint_positions[1], joint_positions[2],
                        joint_positions[3] / M_PI * 180, joint_positions[4] / M_PI * 180, joint_positions[5] / M_PI * 180);

            CartesianPose pose_right;
            pose_right.tran.x = joint_positions[6];
            pose_right.tran.y = joint_positions[7];
            pose_right.tran.z = joint_positions[8];
            pose_right.rpy.rx = joint_positions[9];
            pose_right.rpy.ry = joint_positions[10];
            pose_right.rpy.rz = joint_positions[11];
            RCLCPP_INFO(this->get_logger(), "servo p pose_right: %f, %f, %f, %f, %f, %f",
                        joint_positions[6], joint_positions[7], joint_positions[8],
                        joint_positions[9] / M_PI * 180, joint_positions[10] / M_PI * 180, joint_positions[11] / M_PI * 180);

            robot_->edg_servo_p(&pose_left, move_mode, 1, 0);
            robot_->edg_servo_p(&pose_right, move_mode, 1, 1);
            int result = robot_->edg_send();
            // robot_->edg_servo_p(1, &pose, move_mode, 1);
            // int result = robot_->edg_send();
            // int result = robot_->edg_servo_j(robot_arm_index, &joint_values, move_mode);
            RCLCPP_INFO(this->get_logger(), "result: %d", result);
            if (result != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to send servo_p command. Error code: %d", result);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    // Subscriber Callbacks
    void servo_j7_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {

        if (!servo_move_enabled_)
        {
            RCLCPP_WARN(this->get_logger(), "Servo Move is not enabled. Please enable it first.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "receive servo j");

        try
        {
            std::vector<double> joint_positions = msg->position;
            MoveMode move_mode;
            if (msg->name.empty())
            {
                RCLCPP_INFO(this->get_logger(), "Exception:  ABS ");
                // move_mode[0] = ABS;
                // move_mode[1] = ABS;
                move_mode = ABS;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Exception:  INCR ");
                // move_mode[0] = INCR;
                // move_mode[1] = INCR;
                move_mode = INCR;
            }

            if (joint_positions.size() != 14)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid number of joint positions: %zu", joint_positions.size());
                return;
            }

            JointValue joint_values[2]{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            ;
            for (int i = 0; i < 7; i++)
            {
                joint_values[0].jVal[i] = joint_positions[i];
                RCLCPP_INFO(this->get_logger(), "Left arm joint %d value: %f", i, joint_values[0].jVal[i]);
                joint_values[1].jVal[i] = joint_positions[i + 7];
                RCLCPP_INFO(this->get_logger(), "Right arm joint %d value: %f", i, joint_values[1].jVal[i]);
            }

            robot_->edg_servo_j(0, &joint_values[0], move_mode, 1);
            robot_->edg_servo_j(1, &joint_values[1], move_mode, 1);
            int result = robot_->edg_send();
            // int result = robot_->edg_servo_j(robot_arm_index, &joint_values, move_mode);
            RCLCPP_INFO(this->get_logger(), "result: %d", result);
            if (result != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to send servo_j7 command. Error code: %d", result);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    // Subscriber Callbacks
    void servo_j_callback_left_arm(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        try
        {
            auto move_mode = msg->name.empty() ? ABS : INCR;

            if (msg->position.size() != 7)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid number of left servo j: %zu", msg->position.size());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "before get left arm command j val");
            dual_command_servo_j_val[0] = msg->position[0];
            dual_command_servo_j_val[1] = msg->position[1];
            dual_command_servo_j_val[2] = msg->position[2];
            dual_command_servo_j_val[3] = msg->position[3];
            dual_command_servo_j_val[4] = msg->position[4];
            dual_command_servo_j_val[5] = msg->position[5];
            dual_command_servo_j_val[6] = msg->position[6];
            left_arm_command_j_coming = true;
            RCLCPP_INFO(this->get_logger(), "after get left arm command j val");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    // Subscriber Callbacks
    void servo_j_callback_right_arm(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        try
        {
            auto move_mode = msg->name.empty() ? ABS : INCR;

            if (msg->position.size() != 7)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid number of right servo j: %zu", msg->position.size());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "before get right arm command j val");
            dual_command_servo_j_val[7] = msg->position[0];
            dual_command_servo_j_val[8] = msg->position[1];
            dual_command_servo_j_val[9] = msg->position[2];
            dual_command_servo_j_val[10] = msg->position[3];
            dual_command_servo_j_val[11] = msg->position[4];
            dual_command_servo_j_val[12] = msg->position[5];
            dual_command_servo_j_val[13] = msg->position[6];
            right_arm_command_j_coming = true;
            RCLCPP_INFO(this->get_logger(), "after get right arm command j val");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    // Subscriber Callbacks
    void servo_p_callback_left_arm(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        try
        {
            auto move_mode = msg->name.empty() ? ABS : INCR;

            if (msg->position.size() != 6)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid number of left servo p: %zu", msg->position.size());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "before get left arm command p val");
            dual_command_servo_p_val[0] = msg->position[0];
            dual_command_servo_p_val[1] = msg->position[1];
            dual_command_servo_p_val[2] = msg->position[2];
            dual_command_servo_p_val[3] = msg->position[3];
            dual_command_servo_p_val[4] = msg->position[4];
            dual_command_servo_p_val[5] = msg->position[5];
            left_arm_command_p_coming = true;
            RCLCPP_INFO(this->get_logger(), "after get left arm command p val");
            dual_arm_command_p_send_callback();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    void servo_p_callback_right_arm(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        try
        {
            auto move_mode = msg->name.empty() ? ABS : INCR;

            if (msg->position.size() != 6)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid number of right servo p: %zu", msg->position.size());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "before get right arm command p val");
            dual_command_servo_p_val[6] = msg->position[0];
            dual_command_servo_p_val[7] = msg->position[1];
            dual_command_servo_p_val[8] = msg->position[2];
            dual_command_servo_p_val[9] = msg->position[3];
            dual_command_servo_p_val[10] = msg->position[4];
            dual_command_servo_p_val[11] = msg->position[5];
            right_arm_command_p_coming = true;
            RCLCPP_INFO(this->get_logger(), "after get right arm command p val");
            dual_arm_command_p_send_callback();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JakaRobotDriver>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

// int temp()
// {
//     std::cout << "hello" << std::endl;
//     JAKAZuRobot robot;
//     RobotStatus robotStatus;
//     errno_t ret;

//     ret = robot.login_in(IP);
//     ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

//     char version[128];
//     robot.get_sdk_version(version);
//     std::cout << "version: " << version << std::endl;

//     ret = robot.power_on();
//     ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

//     check_error(robot, true);

//     ret = robot.enable_robot();
//     if (ret != ERR_SUCC)
//     {
//         printf("enable failed! ret = %d\n", ret);
//     }

//     check_state(robot);

//     RobotState state;
//     ret = robot.get_robot_state(&state);
//     if (!state.servoEnabled)
//     {
//         ErrorCode code;
//         robot.get_last_error(&code);
//         printf("Robot is error! error ocde = %2x, msg = %s\n", code.code, code.message);
//         robot.power_off();
//         exit(1);
//     }

//     ret = robot.clear_error();
//     robot.set_collision_level(LEFT, 0);
//     robot.set_collision_level(RIGHT, 0);

//     JointValue jstep_pos[2]{{0.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}, {0.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}};
//     JointValue jstep_pos_neg[2]{{-0.2, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1}, {-0.2, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1}};
//     JointValue jstep_neg[2]{{-0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1}, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}};
//     for (int i = 0; i < 1; i++)
//     {
//         MoveMode moveop[2] = {INCR, INCR};
//         double vel[2] = {1, 1};
//         double acc[2] = {1, 1};
//         double tol[2] = {0, 0};
//         double id[2] = {0, 0};
//         ret = robot.robot_run_multi_movj(-1, moveop, FALSE, jstep_pos_neg, vel, acc); // Relative motion
//         // ret = robot.robot_run_multi_movj(-1, moveop, FALSE, jstep_pos_neg, vel, acc);   // Relative motion
//     }
//     while (1)
//     {
//         std::this_thread::sleep_for(std::chrono::milliseconds(500));
//         if (check_inpos(robot))
//         {
//             break;
//         }
//     }

//     // Blocking mode
//     //  for (int i = 0; i < 3; i++)
//     //  {
//     //  MoveMode moveop[2] = {ABS, ABS};
//     //  double vel[2] = {0.1, 0.1};
//     //  double acc[2] = {0.1, 0.1};
//     //  double tol[2] = {0, 0};
//     //  double id[2] = {0, 0};
//     //  ret = robot.robot_run_multi_movj(-1, moveop, TRUE, jstep_pos, vel, acc);
//     //  if (ret != ERR_SUCC)
//     //  {
//     //  std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     //  std::cout << "joint_move pos failed.\n";
//     // }
//     // else
//     // {
//     // std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     // std::cout << "joint_move pos ok.\n";
//     // }

//     // ret = robot.robot_run_multi_movj(-1, moveop, TRUE, jstep_neg, vel, acc);
//     // if (ret == ERR_SUCC)
//     // {
//     // std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     // std::cout << "joint_move neg ok.\n";
//     // }
//     // else
//     // {
//     // std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     // std::cout << "joint_move neg failed.\n";
//     // }
//     // }

//     robot.login_out();
//     return 0;
// }
