/**************************************************************************
 * 
 * Copyright (c) 2024 JAKA Robotics, Ltd. All Rights Reserved.
 * 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * For support or inquiries, please contact support@jaka.com.
 * 
 * File: JAKAZuRobot.h
 * @author star@jaka
 * @date Nov-30-2021 
 *  
**************************************************************************/


#ifndef _JAKAAPI_H_
#define _JAKAAPI_H_

#include <stdio.h>
#include <string>
#include <stdint.h>
#include "jkerr.h"
#include "jktypes.h"

#if defined(_WIN32) || defined(WIN32)
/**
    * @brief Constructor for the robotic arm control class
 */
#if __cpluscplus

#ifdef DLLEXPORT_API
#undef DLLEXPORT_API
#endif // DLLEXPORT_API

#ifdef DLLEXPORT_EXPORTS
#define DLLEXPORT_API __declspec(dllexport)
#else // DLLEXPORT_EXPORTS
#define DLLEXPORT_API __declspec(dllimport)
#endif // DLLEXPORT_EXPORTS

#else // __cpluscplus

#define DLLEXPORT_API

#endif // __cpluscplus

#elif defined(__linux__)

#define DLLEXPORT_API __attribute__((visibility("default")))

#else

#define DLLEXPORT_API

#endif // defined(_WIN32) || defined(WIN32)
class JAKARobotInterface;
class DLLEXPORT_API JAKAZuRobot
{
public:
	/**
	* @brief Robotic arm control class constructor
	*/
	JAKAZuRobot();

///@name general part
///@{
	/**
    * @brief Create a control handle for the robot
	*
    * @param ip IP address of the controller
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t login_in(const char *ip, bool grpc_flag = false);
	/**
    * @brief Disconnect the controller. The connection with the cobot will be then terminated.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t login_out();

	/**
    * @brief Power on the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t power_on();

	/**
	* @brief Power off the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t power_off();

	/**
    * @brief  Shut down the control cabinet. The cobot must be disabled and powered off before executing this command.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t shut_down();

	/**
    * @brief Enable the cobot. The cobot must be powered on before executing this command.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t enable_robot();

	/**
    * @brief Disable the robot
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t disable_robot();

	/**
    * @brief Get current status of the cobot(A large struct contains as more data as possible).
	* @deprecated please check each inner data using corresponding interfaces instead of this one. 
	* @warning please note that config of OptionalInfoConfig.ini may affect data from port 10004 and data in RobotStatus without error
	* if you're not familiar with this, please keep OptionalInfoConfig.ini no changed as defalut setting. 
	*
    * @param status Robot status
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_robot_status(RobotStatus *status);

	/**
	* @brief Get current status of the cobot(A small struct only contains error/poweron/enable info).
	*
	* @param status Pointer for the returned cobot status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_robot_status_simple(RobotStatus_simple *status);

	/**
	* @brief Get current status of the cobot
	* @deprecated please use "get_robot_status_simple" instead
	*
	* @param state Pointer for the returned cobot status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_robot_state(RobotState* state);

	/**
    * @brief Get the Denavit–Hartenberg parameters of the cobot.
	*
	* @param dhParam Pointer of a varible to save the returned Denavit–Hartenberg parameters.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_dh_param(DHParam* dh_param);

	/**
	* @brief Set installation (or mounting) angle of the cobot.
	*
    * @param angleX Rotation angle around the X-axis
    * @param angleZ Rotation angle around the Z-axis
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_installation_angle(double angleX, double angleZ);

	/**
	* @brief Get installation (or mounting) angle of the cobot.
	*
	* @param quat Pointer for the returned result in quaternion.
	* @param appang Pointer for the returned result in RPY.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_installation_angle(Quaternion* quat, Rpy* appang);

	/**
	* @brief Registor Callback funtion used when controller raise error	
	* @deprecated Only useful before SDK 2.1.12, check return code of each interface instead of using this
	*
	* @param func Callback function
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_error_handler(CallBackFuncType func);
///@}
	
///@name motion part
///@{

    /**
	* @brief set motion planner
	* 
	* @param type: 0: T, 1: S
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
    errno_t set_motion_planner(MotionPlannerType type);

	/**
	* @brief Set timeout of motion block
	* @deprecated  Used only in SDK version before v2.1.12
	* 
	* @param seconds Timeout, unit: second
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_block_wait_timeout(float seconds);

	/**
	* @brief Set time interval of getting data via port 10004
	* @deprecated  Used only in SDK version before v2.1.12
	*
	* @param millisecond Time interval, unit: millisecond
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_status_data_update_time_interval(float millisecond);

	/**
	* @brief Jog a single joint or axis of the cobot to move in specified mode
	*
    * @param aj_num 0-based axis or joint number, indicating joint number 0-5 in joint space, and x, y, z, rx, ry, rz in Cartesian space
    * @param move_mode Robot motion mode, incremental motion or continuous motion
    * @param coord_type Robot motion coordinate system, tool coordinate system, base coordinate system (current world/user coordinate system) or joint space
    * @param vel_cmd Commanded velocity, unit rad/s for rotary axis or joint motion, mm/s for moving axis
    * @param pos_cmd Commanded position, unit rad for rotary axis or joint motion, mm for moving axis
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t jog(int aj_num, MoveMode move_mode, CoordType coord_type, double vel_cmd, double pos_cmd);

	/**
	* @brief Stop the ongoing jog movement.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t jog_stop(int num);

	/**
	* @brief Move the cobot in joint space, without consideration of the path of TCP in Cartesian space.
	*
    * @param joint_pos Target position for robot joint motion
    * @param move_mode Specifies the motion mode: incremental motion (relative motion) or absolute motion
    * @param is_block Set whether the interface is a blocking interface, TRUE for blocking interface, FALSE for non-blocking interface
    * @param speed Robot joint motion speed, unit: rad/s
    * @param acc Robot joint motion angular acceleration, unit: rad/s^2
    * @param tol Robot joint motion end point error, unit: mm
    * @param option_cond Optional parameters for robot joints, if not needed, the value can be left unassigned, just fill in a null pointer
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t joint_move(const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed, double acc = 3.5, double tol = 0, const OptionalCond *option_cond= nullptr);

	/**
	* @brief Move the cobot in Cartesian space, the TCP will move linearly.
	*
	* @param end_pos The target position of the robot's end motion.
	* @param move_mode Specify the motion mode: incremental (relative) or absolute.
	* @param is_block Set if the interface is a blocking interface, TRUE for blocking interface FALSE for non-blocking interface.
	* @param speed Robot linear motion speed, unit: mm/s
	* @param acc Acceleration of the robot in mm/s^2.
	* @param tol The robot's endpoint error in mm.
	* @param option_cond robot joint optional parameters, if not needed, the value can not be assigned, fill in the empty pointer can be
	* @param ori_vel Attitude velocity, unit rad/s
	* @param ori_acc Attitude acceleration, unit rad/s^2.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t linear_move(const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed, double accel = 500, double tol = 0, const OptionalCond *option_cond = nullptr, double ori_vel=3.14, double ori_acc=12.56);

	/**
	* @brief Move the cobot in Cartesan space and the TCP will move along the definded arc.
	*
	* @param end_pos The target position of the robot's end motion.
	* @param mid_pos The midpoint of the robot's end motion.
	* @param move_mode Specify the motion mode: incremental (relative) or absolute.
	* @param is_block Set if the interface is a blocking interface, TRUE for blocking interface FALSE for non-blocking interface.
	* @param speed Command speed of the linear movement, in unit mm/s.
	* @param acc linear acceleration command for the movement, in unit mm/s^2
	* @param tol endpoint error of the robot's circular motion, in millimeters.
	* @param option_cond Optional parameter for robot joint, if not needed, this value can not be assigned, just fill in the empty pointer.
	* @param circle_cnt Specifies the number of circles of the robot. A value of 0 is equivalent to circle_move.
	* @param circle_mode Specifies the mode of the robot's circular motion, the parameter explanation is as follows:
	- 0: Fixed to use the axis angle of rotation angle less than 180° from the start attitude to the end attitude for attitude change; (current program)
	- 1: Fixedly adopts the axis angle of the rotation angle from the start attitude to the termination attitude which is greater than 180° for attitude change;
	- 2: Selection of whether the angle is less than 180° or more than 180° is automatically chosen according to the midpoint attitude;
	- 3: The attitude pinch angle is always consistent with the arc axis. (Current whole circle motion)
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t circular_move(const CartesianPose *end_pos, const CartesianPose *mid_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond = nullptr, double circle_cnt = 0, int circle_mode = 0);


	/**
	* @brief Setting the robot run rate
	*
	* @param rapid_rate Value of the velociry rate, range from [0,1].
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_rapidrate(double rapid_rate);

	/**
	* @brief Get the robot runtime rate
	*
	* @param rapid_rate Pointer for the returned current velocity rate.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_rapidrate(double *rapid_rate);

	/**
	* @brief Define the data of tool with specified ID.
	*
	* @param id ID of the tool to be defined.
	* @param tcp Data of the tool to be set.
	* @param name Alias name of the tool to be set.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_tool_data(int id, const CartesianPose *tcp, const char *name);

	/**
	* @brief Switch to the tool with specified ID.
	*
	* @param id ID of the tool.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_tool_id(const int id);

	/**
	* @brief Get ID of the tool currently used.
	*
	* @param id Pointer for the returned current tool ID.		
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tool_id(int *id);

	/**
	* @brief Get definition data of the tool with specified ID.
	*
	* @param id ID of the tool.
	* @param tcp Pointer for the returned tool data	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tool_data(int id, CartesianPose *tcp);

	/**
	* @brief Define the data of user frame with specified ID.
	*
	* @param id ID of the user frame.
	* @param user_frame Data of the user frame to be set.
	* @param name Alias	name of the user frame.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_user_frame_data(int id, const CartesianPose *user_frame, const char *name);

	/**
	* @brief Switch to the user frame with specified ID.
	*
	* @param id ID of the user frame.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_user_frame_id(const int id);

	/**
	* @brief Get ID of the user frame currently used.
	*
	* @param id Pointer for the returned current user frame ID.		
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_user_frame_id(int *id);

	/**
	* @brief Get definition data of the user frame with specified ID.
	*
	* @param id ID of the user frame.
	* @param tcp Pointer for the returned user frame data	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_user_frame_data(int id, CartesianPose *user_frame);

	/**
	* @brief Set payload for the cobot.
	*
	* @param payload payload data to be set.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_payload(const PayLoad *payload);

	/**
	* @brief Get current payload of the cobot.
	*
	* @param payload Pointer for the returned payload data.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_payload(PayLoad *payload);

	/**
	* @brief Get the position of the end of the tool in the current setting.
	*
	* @param tcp_position Pointer for the returned TCP position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tcp_position(CartesianPose *tcp_position);

	/**
	* @brief Get current joint position.
	*
	* @param joint_position Pointer for the return joint position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_joint_position(JointValue *joint_position);
	/**
	* @brief Get the position of the end of the tool in the actual setting.
	*
	* @param tcp_position Pointer for the returned TCP position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
    errno_t get_actual_tcp_position(CartesianPose *tcp_position);
	/**
	* @brief Get actual joint position.
	*
	* @param joint_position Pointer for the return joint position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
    errno_t get_actual_joint_position(JointValue *joint_position);

	/**
	 * @brief Check if the cobot is now in E-Stop state.
	 *
	 * @param estop Pointer for the returned result.	
	 *
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */	
	errno_t is_in_estop(BOOL *estop);

	/**
	* @brief Check if the cobot is now on soft limit.
	*
	* @param on_limit Pointer for the returned result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t is_on_limit(BOOL *on_limit);

	/**
	* @brief Check if the cobot has completed the motion (does not mean reached target).
	*
	* @param in_pos Pointer for the returned result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t is_in_pos(BOOL *in_pos);

	/**
	* @brief Stop all the ongoing movements of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t motion_abort();
	
	/**
	* @brief Get motion status of the cobot.
	*
	* @param status Pointer for the returned motion status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	errno_t get_motion_status(MotionStatus *status);
	/**
	* @brief .
	*
	* @param joint .
	* @param gain 
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	errno_t set_drag_friction_compensation_gain(int joint, int gain);
	/**
	* @brief .
	*
	* @param list .
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
    errno_t get_drag_friction_compensation_gain(DragFrictionCompensationGainList* list);

///@}

///@name TIO part
///@{

	/**
	* @brief Set voltage parameter for TIO of the cobot. It only takes effect for TIO with hardware version 3.
	*
	* @param vout_enable Option to enable voltage output. 0:turn off， 1:turn on.
	* @param vout_vol Option to set output voltage. 0: 24V, 1:12V.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_tio_vout_param(int vout_enable, int vout_vol);

	/**
	* @brief Get voltage parameter of TIO of the cobot. It only takes effect for TIO with hardware version 3.
	* 
	* @param vout_enable Pointer for the returned voltage enabling option.
	* @param vout_vol Pointer for the returned output voltage option.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tio_vout_param(int* vout_enable, int* vout_vol);

	/**
	* @brief Add or modify the signal for TIO RS485 channels.
	*
	* @param sign_info Definition data of the signal.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t add_tio_rs_signal(SignInfo sign_info);

	/**
	* @brief Delete the specified signal for TIO RS485 channel.
	*
	* @param sig_name Signal name.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t del_tio_rs_signal(const char* sig_name);

	/**
	* @brief Send a command to the specified RS485 channel.
	*
	* @param chn_id ID of the RS485 channel in TIO.
	* @param data Command data.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t send_tio_rs_command(int chn_id, uint8_t* data,int buffsize);

	/**
	* @brief Get all the defined signals in TIO module.
	* 
	* @param sign_info_array Pointer for the returned signal list.
	* @param array_len Pointer for the size of the returned signal list.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_rs485_signal_info(SignInfo* sign_info_array, int* array_len);

	/**
	* @brief Set tio mode.
	*
	* @param pin_type tio type 0 for DI Pins, 1 for DO Pins, 2 for AI Pins
	* @param pin_type tio mode DI Pins: 0:0x00 DI2 is NPN,DI1 is NPN,1:0x01 DI2 is NPN,DI1 is PNP, 2:0x10 DI2 is PNP,DI1 is NPN,3:0x11 DI2 is PNP,DI1 is PNP
							 DO Pins: Low 8-bit data high 4-bit for DO2 configuration, low 4-bit for DO1 configuration, 0x0 DO for NPN output, 0x1 DO for PNP output, 0x2 DO for push-pull output, 0xF RS485H interface
							 AI Pins: 0: analog input function enable, RS485L disable, 1: RS485L interface enable, analog input function disable
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_tio_pin_mode(int pin_type, int pin_mode);

	/**
	* @brief Get mode of the TIO pin of specified type.
	*
	* @param pin_type tio type 0 for DI Pins, 1 for DO Pins, 2 for AI Pins
	* @param pin_mode tio mode DI Pins: 0:0x00 DI2 is NPN,DI1 is NPN,1:0x01 DI2 is NPN,DI1 is PNP, 2:0x10 DI2 is PNP,DI1 is NPN,3:0x11 DI2 is PNP,DI1 is PNP
							 DO Pins: Low 8-bit data high 4-bit for DO2 configuration, low 4-bit for DO1 configuration, 0x0 DO for NPN output, 0x1 DO for PNP output, 0x2 DO for push-pull output, 0xF RS485H interface
							 AI Pins: 0: analog input function enable, RS485L disable, 1: RS485L interface enable, analog input function disable
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tio_pin_mode(int pin_type, int* pin_mode);

	/**
	* @brief Setup communication for specified RS485 channel.
	*
	* @param ModRtuComm When the channel mode is set to Modbus RTU, you need to specify the Modbus slave node ID additionally.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_rs485_chn_comm(ModRtuComm mod_rtu_com);

	/**
	* @brief Get RS485 commnunication setting.
	* 
	* @param mod_rtu_com Pointer for the returned communication settings.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_rs485_chn_comm(int chn_id,ModRtuComm* mod_rtu_com);

	/**
	* @brief Set the mode for specified RS485 channel.
	*
	* @param chn_id Channel id. 0 for RS485H, channel 1; 1 for RS485L, channel 2.
	* @param chn_mode Mode to indicate the usage of RS485 channel. 0 for Modbus RTU, 1 for Raw RS485, 2 for torque sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_rs485_chn_mode(int chn_id, int chn_mode);

	/**
	* @brief Get the mode of specified RS485 channel.
	* 
	* @param chn_id Channel id. 0: RS485H, channel 1; 1: RS485L, channel 2
	* @param chn_mode Pointer for the returned mode. 0: Modbus RTU, 1: Raw RS485, 2, torque sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_rs485_chn_mode(int chn_id, int* chn_mode);
	
	/**
	* @brief Perform hardware-level calibration of the TIO sensor.
	* 
	* @param type 0: No effect, 1: Perform calibration, 2: Query the current calibration value.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t tio_sensor_calib(int type);

	/**
	* @brief Update the signal for TIO RS485 channels.
	*
	* @param sign_info Definition data of the signal.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t update_tio_rs_signal(SignInfo_simple sign_info);

///@}

///@name Trajectory recording & replay
///@{

	/**
	* @brief Set trajectory recording parameters for the cobot.
	*
	* @param para Trajectory recording parameters.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_traj_config(const TrajTrackPara *para);

	/**
	* @brief Get trajectory recording parameters of the cobot.
	*
	* @param para Pointer for the returned trajectory recording parameters.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_traj_config(TrajTrackPara *para);

	/**
	* @brief Acquisition track reproduction data control switch
	*
	* @param mode Trajectory recording (sampling) mode. TRUE to start trajectory recording (sampling) and FALSE to disable.
	* @param filename File name	to save the trajectory recording results.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_traj_sample_mode(const BOOL mode, char *filename);

	/**
	* @brief Get current trajectory recording status of the cobot.
	*
	* @param mode Pointer for the returned status.TRUE if it's now recording, FALSE if the data recording is over or not recording. 
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_traj_sample_status(BOOL *sample_status);

	/**
	* @brief Get all the existing trajectory recordings of the cobot.
	*
	* @param filename  Pointer for the returned trajectory recording files.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_exist_traj_file_name(MultStrStorType *filename);

	/**
	* @brief Rename the specified trajectory recording file.
	*
	* @param src Source file name of the trajectory recording.
	* @param dest Dest file name of the trajectory recording, the length must be no more than 100 characters.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t rename_traj_file_name(const char *src, const char *dest);

	/**
	* @brief Delete the specified trajectory file.
	*
	* @param filename File name of the trajectory recording.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t remove_traj_file(const char *filename);

	/**
	* @brief  Generate program scripts from specified trajectory recording file.
	*
	* @param filename Trajectory recording file.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t generate_traj_exe_file(const char *filename);

///@}

///@name servo part
///@{
	/**
	* @brief Enable or disable servo mode for the cobot.
	*
	* @param enable Option to enable or disable the mode. TRUE to enable servo mode，FALSE to disable servo mode.	
	* @param is_block Set whether the interface is a blocking interface, TRUE for blocking interface, FALSE for non-blocking interface
	* @param robot_id ID of the target robot. Defaults to 0 if not specified.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_move_enable(BOOL enable, BOOL is_block = true, int robot_id = 0);

	/**
	 * @brief Check if the cobot is now in servo move mode.
	 *
	 * @param is_servo Pointer for the returned result.
	 * @param robot_id ID of the target robot. Defaults to 0 if not specified.
	 *
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	errno_t is_in_servomove(BOOL *in_servo, int robot_id = 0);

	/**
	* @brief Move the cobot to the specifed joint position in servo mode. It will only work when the cobot is already in servo 
	* move mode. The servo_j command will be processed within one interpolation cycle. To ensure the cobot moves smoothly, 
	* client should send next command immediately to avoid any time delay.
	*
	* @param joint_pos The target position of the robot joint motion.
	* @param move_mode Specify the motion mode: incremental or absolute.
	* @param step_num times the period, servo_j movement period for step_num * 8ms, where step_num> = 1
	* @param do_info Optional pointer to store additional DO information. Defaults to nullptr if not needed.
	* @param robot_id ID of the target robot. Defaults to 0 if not specified.
	* @param queue_num Output pointer to store the current command queue length.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_j(const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num = 1, int *queue_num = nullptr, DOInfo *do_info = nullptr, int robot_id = 0);

	/**
	* @brief Move the cobot to the specifed Cartesian position in servo mode. Simalar with servo_j command, it will only 
	* work when the cobot is already in servo move mode and will be processed within one interpolation cycle. To ensure 
	* the cobot moves smoothly, client should send next command immediately to avoid any time delay.
	*
	* @param cartesian_pose The target position of the robot's Cartesian motion.
	* @param move_mode Specify the motion mode: incremental or absolute.
	* @param step_num times the period, servo_p movement period is step_num * 8ms, where step_num>=1
	* @param do_info Optional pointer to store additional DO information. Defaults to nullptr if not needed.
	* @param robot_id ID of the target robot. Defaults to 0 if not specified.
	* @param queue_num Output pointer to store the current command queue length.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_p(const CartesianPose *cartesian_pose, MoveMode move_mode, unsigned int step_num = 1, int *queue_num = nullptr, DOInfo *do_info = nullptr, int robot_id = 0);
	
	/**
	* @brief Enable the EDG(external data guider) function.The EDG-related interfaces can only be used after enabling this function.
	*
	* @param en  Enable switch. true enables the EDG function, while false disables the function.
	* @param edg_stat_ip The IP address of the SDK client.
	* @param edg_port The SDK client port for receiving EDG feedback data.
	* @param edg_mode Specify the edg mode: 0:All EDG-related interfaces can be called, 1:All interfaces except edg_servo_j and edg_servo_p can be called.
	* @param edg_prio The priority of the EDG thread, only for linux.
	* @param edg_cpuid The CPU ID of the EDG thread, only for linux.
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t edg_init(BOOL en = true, const char* edg_stat_ip = "0.0.0.0", int edg_port = 10010, int edg_mode = 0, int prio = 85, int cpuid = 3);

	/**
	* @brief Move the cobot to the specifed joint position in servo mode. It will only work when the cobot is already in servo 
	* move mode. 

	* @param joint_pos The target position of the robot joint motion.
	* @param move_mode Specify the motion mode: incremental or absolute.
	* @param step_num times the period, servo_j movement period for step_num * 8ms, where step_num> = 1
	* @param robot_index Uses the default value, and no parameter needs to be passed.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t edg_servo_j(const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num = 1, unsigned char robot_index = 0);	
	
	/**
	* @brief Move the cobot to the specifed Cartesian position in servo mode. 
	*
	* @param cartesian_pose The target position of the robot's Cartesian motion.
	* @param move_mode Specify the motion mode: incremental or absolute.
	* @param step_num times the period, servo_p movement period is step_num * 8ms, where step_num>=1
	* @param robot_index Uses the default value, and no parameter needs to be passed.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t edg_servo_p(const CartesianPose *cartesian_pose, MoveMode move_mode, unsigned int step_num = 1, unsigned char robot_index = 0);

	/**
	* @brief High-speed acquisition of EDG feedback data from the cobot. 
	*
	* @param edg_state EDG feedback data (including joint positions, velocities, torques, Cartesian positions, CAB IO, tool IO, and sensor data).
	* @param robot_index Uses the default value, and no parameter needs to be passed.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t edg_get_stat(EDGState *edg_state, unsigned char robot_index = 0);

	/**
	* @brief Get the timestamp of when the EDG feedback data was issued.
	*
	* @param details Timestamp
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t edg_stat_details(unsigned long int details[3]);

	/**
	* @brief Disable the filter for servo move commands.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	* @param robot_id ID of the target robot. Defaults to 0 if not specified.
	*/
	errno_t servo_move_use_none_filter(int robot_id = 0);

	/**
	* @brief Set 1st-order low-pass filter for servo move. It will take effect for both servo_j and servo_p commands.
	*
	* @param cutoffFreq Cut-off frequency for low-pass filter.
	* @param robot_id ID of the target robot. Defaults to 0 if not specified.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_move_use_joint_LPF(double cutoffFreq, int robot_id = 0);

	/**
	* @brief Set 3rd-order non-linear filter in joint space for servo move. It will take effect for both servo_j and 
	* servo_p commands.
	*
	* @param max_vr Joint speed limit, in unit deg/s
	* @param max_ar Joint acceleration limit, in unit deg/s^2
	* @param max_jr Joint jerk limit, in unit deg/s^3	
	* @param robot_id ID of the target robot. Defaults to 0 if not specified.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_move_use_joint_NLF(double max_vr, double max_ar, double max_jr, int robot_id = 0);

	/**
	* @brief Set 3rd-order non-linear filter in Cartesian space for servo move. It will only take effect for servo_p
	* since the filter will be applied to Cartesian position in the commands.
	*
	* @param max_vp Speed limit, in unit mm/s.s
	* @param max_ap Acceleration limit, in unit mm/s^2.
	* @param max_jp Jerk limit, in unit mm/s^3.
	* @param max_vr Orientation speed limit, in unit deg/s.
	* @param max_ar Orientation acceleration limit, in unit deg/s^2.
	* @param max_jr Orientation jerk limit, in unit deg/s^3.
	* @param robot_id ID of the target robot. Defaults to 0 if not specified.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_move_use_carte_NLF(double max_vp, double max_ap, double max_jp, double max_vr, double max_ar, double max_jr, int robot_id = 0);

	/**
	* @brief Set multi-order mean filter in joint space for servo move. It will take effect for both servo_j and servo_p
	* commands.
	*
	* @param max_buf Indicates the size of the mean filter buffer. If the filter buffer is set too small (less than 3), it
	* is likely to cause planning failure. The buffer value should not be too large (>100), which will bring computational
	* burden to the controller and cause planning delay; as the buffer value increases, the planning delay time increases.
	* @param kp Position filter coefficient.
	* @param kv Velocity filter coefficient.
	* @param ka Acceleration filter coefficient.
	* @param robot_id ID of the target robot. Defaults to 0 if not specified.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_move_use_joint_MMF(int max_buf, double kp, double kv, double ka, int robot_id = 0);

	/**
	* @brief Set velocity look-ahead filter. It’s an extended version based on the multi-order filtering algorithm with 
	* look-ahead algorithm, which can be used for joints data and Cartesian data.
	* 
	* @param max_buf Buffer size of the mean filter. A larger buffer results in smoother results, but with higher precision
	* loss and longer planning lag time.
	* @param kp Position filter coefficient. Reducing this coefficient will result in a smoother filtering effect, but a 
	* greater loss in position accuracy. Increasing this coefficient will result in a faster response and higher accuracy,
	* but there may be problems with unstable operation/jitter, especially when the original data has a lot of noise.
	* @param robot_id ID of the target robot. Defaults to 0 if not specified.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_speed_foresight(int max_buf, double kp, int robot_id = 0);

///@}

///@name IO part
///@{
	/**
	* @brief Set the value of a digital output (DO).
	*
	* @param type DO type
	* @param index DO index
	* @param value DO set value
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_digital_output(IOType type, int index, BOOL value);

	/**
	* @brief Set the value of the analog output (AO).
	*
	* @param type AO type.
	* @param index AO index.
	* @param value AO set value.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_analog_output(IOType type, int index, float value);

	/**
	* @brief Query Digital Input (DI) Status.
	*
	* @param type DI type.
	* @param index DI index.
	* @param result DI status query result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_digital_input(IOType type, int index, BOOL *result);

	/**
	* @brief Query digital output (DO) status.
	*
	* @param type DO type.
	* @param index DO index.
	* @param result DO status query result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_digital_output(IOType type, int index, BOOL *result);

	/**
	* @brief Get the value of the analog input (AI).
	*
	* @param type The type of AI.
	* @param index AI index.
	* @param result Specify the result of AI status query.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_analog_input(IOType type, int index, float *result);

	/**
	* @brief Get the value of analog output (AO).
	*
	* @param type The type of AO.
	* @param index AO index.
	* @param result Specify the result of AO status query.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_analog_output(IOType type, int index, float *result);

	/**
	* @brief Set multiple digital outputs (DO), specified number of DOs starting from certain index will be set.
	*
	* @param type Type of the digital output.
	* @param index Starting index of the digital outputs to be set.
	* @param value Array of the target value.
	* @param len Number of DO to be set.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_digital_output(IOType type, int index, BOOL* value, int len);

	/**
	* @brief Set multiple analog outputs (AO), specified number of AOs starting from certain index will be set.
	*
	* @param type Type of the analog output.
	* @param index Starting index of the analog outputs to be set.
	* @param value Array of the target value.
	* @param len Number of AO to be set.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_analog_output(IOType type, int index, float* value, int len);

	/**
	* @brief Get current status of multiple digital inputs (DI), specified number of DIs starting from certain index will be retrieved.
	*
	* @param type Type of the digital input.
	* @param index Starting index of the digital input.
	* @param result Pointer for the returned DI status.
	* @param len Number of DI to be queried.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_digital_input(IOType type, int index, BOOL *result, int len);

	/**
	* @brief Get current status of multiple digital inputs (DO), specified number of DOs starting from certain index will be retrieved.
	*
	* @param type Type of the digital output.
	* @param index Starting index of the digital outputs.
	* @param result Pointer for the returned DO status.
	* @param len Number of DO to be queried.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_digital_output(IOType type, int index, BOOL *result, int len);

	/**
	* @brief Get current value of multiple analog inputs (AI), specified number of AIs starting from certain index will be retrieved.
	*
	* @param type The type of AI
	* @param index Starting index of the analog inputs.
	* @param result Pointer for the returned AI status.
	* @param len Number of AI to be queried.

	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_analog_input(IOType type, int index, float *result, int len);

	/**
	* @brief Get the value of analog output variable (AO)
	*
	* @param type The type of AO.
	* @param index Starting index of the analog outputs.
	* @param result Pointer for the returned AO status.
	* @param len Number of AO to be queried.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_analog_output(IOType type, int index, float *result, int len);

	/**
	* @brief Check if the extended IO modules are running.
	*
	* @param is_running Pointer for the returned result.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t is_extio_running(BOOL *is_running);

	/**
	 * @brief trig IO during motion, used before next motion cmd
	 * 
	 * @param rel, 0: relative to start point, 1: relate to end point
	*/
	errno_t set_motion_digital_output(IOType type, int index, BOOL value, BOOL rel, double distance);

	/**
	 * @brief trig IO during motion, used before next motion cmd
	 * 
	 * @param rel, 0: relative to start point, 1: relate to end point
	*/
	errno_t set_motion_analog_output(IOType type, int index, float value, BOOL rel, double distance);

///@}

///@name program part
///@{

    /**
	@brief get current program info
	*/
    errno_t get_program_info(ProgramInfo *info);

	/**
	* @brief Start the program for the cobot. It will work only when one program has been loaded.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t program_run();

	/**
	* @brief Pause the ongoing program for the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t program_pause();

	/**
	* @brief Resume the program for the cobot. It will work only when one program is now in paused status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t program_resume();

	/**
	* @brief Abort the ongoing tasks of the cobot, the program or any movement will be terminated.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t program_abort();

	/**
	* @brief Load a program for the cobot.
	*
	* @param file The path of the program. For example: A/A.jks, the <file> is "A"
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t program_load(const char *file);

	/**
	* @brief Get the name of the loaded job program.
	*
	* @param file Pointer for the returned loaded program path. For example: A/A.jks, the <file> is "A"
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_loaded_program(char *file);
	errno_t get_loaded_program(char *file, uint8_t size);

	/**
	* @brief Get current execting line. It's the line number of the motion command for the program scripts. For movement 
	* commands sent via SDK, it's the motion ID defined in the movement.
	*
	* @param curr_line The current line number query result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_current_line(int *curr_line);

	/**
	* @brief Get the status of the robot's program execution.
	*
	* @param status Pointer for the returned program status.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_program_state(ProgramState *status);

	/**
	* @brief Get the user defined variables.
	 *
	 * @param vlist Pointer for the returned list of user defined variables.	
	 
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	errno_t get_user_var(UserVariableList* vlist);

	/**
	 * @brief Set the specified user defined variable.
	 *
	 * @param v Data of the user defined variable to be set, including the ID, value and alias name.	
	 *
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	errno_t set_user_var(UserVariable v);

///@}

///@name Hand-guiding (drag)
///@{
	/**
	* @brief Enable/disable the hand-guiding (drag) mode.
	*
	* @param enable TRUE to enter drag and drop mode, FALSE to exit drag and drop mode.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t drag_mode_enable(BOOL enable);

	/**
	* @brief Check if the cobot is in hand-guiding (drag) mode.
	*
	* @param in_drag Pointer for the returned result.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t is_in_drag_mode(BOOL *in_drag);
///@}

///@name collision part
///@{

	/**
	* @brief Check if the cobot is in collision state.
	*
	* @param in_collision Pointer for the returned result.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t is_in_collision(BOOL *in_collision);

	/**
	* @brief Recover the cobot from collision state. It will only take effect when a cobot is in collision state.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t collision_recover();

	/**
	* @brief Clear error status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t clear_error();

	/**
	* @brief Set the collision sensitivity level for the cobot.
	*
	* @param level  Collision sensitivity level, which is an integer value that ranges from [0-5]:
					0: disable collision detect
					1: collision detection threshold 25N，
					2: collision detection threshold 50N，
					3: collision detection threshold 75N，
					4: collision detection threshold 100N，
					5: collision detection threshold 125N	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_collision_level(const int level);

	/**
	* @brief Get current collision sensitivity level of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_collision_level(int *level);


///@}

///@name math
///@{
	/**
	* @brief Calculate the inverse kinematics for a Cartesian position. It will be calclulated with the current tool, current 
	* mounting angle, and current user coordinate.
	*
	* @param ref_pos Reference joint position for solution selection, the result will be in the same solution space with
	* the reference joint position. 
	* @param cartesian_pose Cartesian position to do inverse kinematics calculation.
	* @param joint_pos Pointer for the returned joint position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t kine_inverse(const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos);

	/**
	* @brief Calculate the forward kinematics for a joint position. It will be calclulated with the current tool, current 
	* mounting angle, and current user coordinate.
	*
	* @param joint_pos The position of the joint in joint space.
	* @param cartesian_pose Cartesian space pose calculation result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t kine_forward(const JointValue *joint_pos, CartesianPose *cartesian_pose);

		/**
	* @brief Calculate the inverse kinematics for a Cartesian position. It will be calclulated with the current tool, current 
	* mounting angle, and current user coordinate.
	*
	* @param ref_pos Reference joint position for solution selection, the result will be in the same solution space with
	* the reference joint position. 
	* @param cartesian_pose Cartesian position to do inverse kinematics calculation.
	* @param joint_pos Pointer for the returned joint position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t kine_inverse(const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos, const CartesianPose* tool, const CartesianPose* userFrame);

	/**
	* @brief Calculate the forward kinematics for a joint position. It will be calclulated with the current tool, current 
	* mounting angle, and current user coordinate.
	*
	* @param joint_pos The position of the joint in joint space.
	* @param cartesian_pose Cartesian space pose calculation result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t kine_forward(const JointValue *joint_pos, CartesianPose *cartesian_pose, const CartesianPose* tool, const CartesianPose* userFrame);

	/**
	* @brief Convert an Euler angle in RPY to rotation matrix.
	*
	* @param rpy The Euler angle data to be converted.
	* @param rot_matrix The converted rotation matrix.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t rpy_to_rot_matrix(const Rpy *rpy, RotMatrix *rot_matrix);

	/**
	* @brief Convert a rotation matrix to Euler angle in RPY
	*
	* @param rot_matrix The rotation matrix data to be converted.
	* @param rpy The result of the converted RPY Euler angles.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t rot_matrix_to_rpy(const RotMatrix *rot_matrix, Rpy *rpy);

	/**
	* @brief Convert a quaternion to rotation matrix.
	*
	* @param quaternion The quaternion data to be converted.
	* @param rot_matrix Pointer for the returned rotation matrix.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t quaternion_to_rot_matrix(const Quaternion *quaternion, RotMatrix *rot_matrix);

	/**
	* @brief Conversion of a rotation matrix to quaternions
	*
	* @param rot_matrix The rotation matrix to be converted.
	* @param quaternion Pointer for the returned quaternion.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t rot_matrix_to_quaternion(const RotMatrix *rot_matrix, Quaternion *quaternion);


///@}

///@name SDK support
///@{

	/**
	* @brief Set the reaction behavior of the cobot when connection to SDK is lost.
	*
	* @param millisecond Timeout of connection loss, unit: ms.
	* @param mnt Reaction behavior type.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_network_exception_handle(float millisecond, ProcessType mnt);

	/**
	* @brief Get the version number of SDK.
	*
	* @param version Pointer for the returned SDK verion.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_sdk_version(char *version);

	/**
	* @brief Get the IP address of the control cabinet.
	*
	* @param controller_name The controller name.
	* @param ip_list Controller ip list, controller name for the specific value to return the name of the corresponding controller IP address, controller name is empty, return to the segment class of all the controller IP address
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_controller_ip(char *controller_name, char *ip_list);

	/**
	* @brief Set the path to the error code file, if you need to use the get_last_error interface you need to set the path to the error code file, if you don't use the get_last_error interface, you don't need to set the interface.
	*
	* @param path  File path for the error code.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_errorcode_file_path(char *path);

	/**
	* @brief Get the last error code of the robot, the last error code will be cleared when clear_error is called.
	*
	* @param code Pointer for the returned error code.
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_last_error(ErrorCode *code);

	/**
	* @brief Enable or disable the debug mode for SDK control. If enabled, SDK log may contains more detailed information. 
	* @deprecated Only useful before SDK 2.1.12
	*
	* @param mode Option to enable or disable the debug mode. 1: enable, 0: disable.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_debug_mode(BOOL mode);

	/**
	* @brief Get the SDK log path.
	*
	* @param filepath Pointer for the returned path.
	* @param size Size of char* buffer
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_SDK_filepath(char* path, int size);

	/**
	* @brief Set file path for the SDK log.
	*
	* @param filepath File path of the log.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_SDK_filepath(const char *filepath);

	/**
	* @brief get SDK log path.
	*
	* @param path Path of SDK log.
	* @param size Size of char* buffer.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	static errno_t static_Get_SDK_filepath(char* path, int size);

	/**
	* @brief Set file path for the SDK log.
	*
	* @param filepath File path of the log.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	static errno_t static_Set_SDK_filepath(const char *filepath);



///@}

///@name Torque sensor common
///@{

	/**
 	* @brief Retrieve basic information of configured force-torque sensors for the specified robot.
 	*
 	* This function retrieves the basic configuration and status of multiple force-torque sensors
 	* associated with a robot. The retrieved data is stored in the provided FTSensorBasicInfoStr structure.
 	*
 	* @param infos Pointer to FTSensorBasicInfoStr structure where the retrieved data will be stored.
 	* @param robot_id ID of the target robot. Defaults to 0 if not specified.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
 	errno_t get_ft_sensor_basic_info(FTSensorBasicInfoStr *infos, int robot_id = 0);
	
	/**
 	* @brief Retrieve soft limit rules for force-torque sensors of a robot.
	*
 	* This function queries the configured soft limit rules for the force-torque sensors of the specified robot.
 	*
 	* @param rules Pointer to FTSensorSoftLimitRuleStr to store the retrieved rules.
 	* @param robot_id ID of the robot to query. Defaults to 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
 	errno_t get_ft_sensor_soft_limit_rules(FTSensorSoftLimitRuleStr *rules, int robot_id = 0);

	/**
 	* @brief Set soft limit rules for force-torque sensors of a robot.
 	*
 	* This function configures soft limit rules for the robot's force-torque sensors,
 	* specifying thresholds beyond which protection actions can be triggered.
 	*
 	* @param rules Pointer to FTSensorSoftLimitRuleStr defining the rules.
 	* @param robot_id ID of the robot. Defaults to 0.
 	*
	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t set_ft_sensor_soft_limit_rules(const FTSensorSoftLimitRuleStr *rules, int robot_id = 0);
	
	/**
 	* @brief Set filter parameters for a specified force-torque sensor.
 	*
 	* Applies a low-pass filter to sensor readings to suppress noise.
 	*
 	* @param sensor_id ID of the sensor to configure.
 	* @param filter Pointer to an array of filter parameters.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t set_ft_sensor_filter(int sensor_id, const double *filter);

	/**
 	* @brief Retrieve filter parameters of a specified force-torque sensor.
 	*
 	* This function returns the current filter settings applied to the sensor.
 	*
 	* @param sensor_id ID of the sensor.
 	* @param filter Pointer to an array where the parameters will be stored.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t get_ft_sensor_filter(int sensor_id, double *filter);

	/**
 	* @brief Get the current on/off mode of a force-torque sensor.
 	*
 	* @param sensor_id ID of the sensor to query.
 	* @param mode Pointer to int where the mode will be stored: 1 = on, 0 = off.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t get_ft_sensor_mode(int sensor_id, int *mode);

	/**
 	* @brief Zero the specified force-torque sensor.
 	*
 	* Sets the current reading of the sensor to zero (baseline calibration).
 	*
 	* @param sensor_id ID of the sensor to zero.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t set_ft_sensor_zero(int sensor_id);

	/**
 	* @brief Set contact force threshold for a specified sensor.
 	*
 	* This defines the force/torque thresholds that can trigger a response.
 	*
 	* @param sensor_id ID of the target sensor.
 	* @param threshold Pointer to FTSensorThresholdStr containing threshold values.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t set_contact_force_threshold(int sensor_id, const FTSensorThresholdStr *threshold);

	/**
 	* @brief Retrieve contact force threshold for a specified sensor.
 	*
 	* Returns the currently configured threshold values.
 	*
 	* @param sensor_id ID of the sensor.
 	* @param threshold Pointer to FTSensorThresholdStr where the thresholds will be stored.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t get_contact_force_threshold(int sensor_id, FTSensorThresholdStr *threshold);

	/**
 	* @brief Set the reference point of the force-torque sensor.
 	*
 	* Defines the 3D offset from the tool frame for force computation.
 	*
 	* @param ref_point Pointer to CartesianTran containing [x, y, z] offset in meters.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
 	errno_t set_ft_sensor_ref_point(const CartesianTran *ref_point, int robot_id = 0);
	
	/**
 	* @brief Get the reference point of the force-torque sensor.
 	*
 	* Retrieves the configured tool-frame offset of the sensor.
 	*
 	* @param ref_point Pointer to CartesianTran to receive [x, y, z] offset in meters.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
 	errno_t get_ft_sensor_ref_point(CartesianTran *ref_point, int robot_id = 0);
	
	/**
 	* @brief Retrieve current force/torque data from a sensor.
 	*
 	* @param sensor_id ID of the sensor.
 	* @param type Type of the data to be retrieved: 1 for actTorque (compatibility interface),  
	*             2 for original readings,  3 for real data without gravity and bias.
 	* @param data Pointer to FTSensorDataStr to receive the result.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t get_ft_sensor_data(int sensor_id, int type, FTSensorDataStr *data);

	/**
 	* @brief Get payload information associated with a sensor.
 	*
 	* @param sensor_id ID of the sensor.
 	* @param payload Pointer to PayLoad structure to store the result.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t get_ft_sensor_payload(int sensor_id, PayLoad * payload);

	/**
 	* @brief Set the payload ID for a specific force-torque sensor.
 	*
 	* Assigns a predefined payload configuration to the sensor.
 	*
 	* @param sensor_id ID of the sensor.
 	* @param payload_id Payload configuration ID to assign.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t set_ft_sensor_payload(int sensor_id, int payload_id);

	/**
 	* @brief Get IDs of force-torque sensors linked to the robot.
 	*
 	* @param sensor_ids Pointer to FTLinkedSensorIDStr to receive results.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
 	errno_t get_linked_ft_sensor_id(FTLinkedSensorIDStr *sensor_ids, int robot_id = 0);
	
	/**
 	* @brief Add and configure a new force-torque sensor.
 	*
 	* @param config Pointer to FTSensorConfigStr containing sensor configuration.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
 	errno_t add_ft_sensor(const FTSensorConfigStr *config, int robot_id = 0);
	
	/**
 	* @brief Remove a force-torque sensor from the system.
 	*
 	* @param sensor_id ID of the sensor to remove.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
 	errno_t remove_ft_sensor(int sensor_id, int robot_id = 0);

	/**
 	* @brief Set velocity limits for force control mode.
 	*
 	* @param limit Pointer to VelocityLimit structure specifying linear and angular limits.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t set_force_ctrl_vel_limit(const VelocityLimit *limit, int robot_id = 0);

	/**
 	* @brief Get velocity limits for force control mode.
 	*
 	* @param limit Pointer to VelocityLimit to receive the limits.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t get_force_ctrl_vel_limit(VelocityLimit *limit, int robot_id = 0);

	/**
 	* @brief Set velocity limits for approaching mode.
 	*
 	* @param limit Pointer to VelocityLimit structure specifying limits.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t set_approaching_vel_limit(const VelocityLimit *limit, int robot_id = 0);

	/**
 	* @brief Get velocity limits for approaching mode.
 	*
 	* @param limit Pointer to VelocityLimit structure to receive limits.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t get_approaching_vel_limit(VelocityLimit *limit, int robot_id = 0);

	/**
 	* @brief Set custom force control configuration.
 	*
 	* @param config Pointer to AdmitCtrlType structure defining control behavior.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t set_cst_force_ctrl_config(const AdmitCtrlType *config, int robot_id = 0);

	/**
 	* @brief Retrieve custom force control configuration.
 	*
 	* @param config Pointer to AdmitCtrlType to receive configuration.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t get_cst_force_ctrl_config(AdmitCtrlType *config, int robot_id = 0);

	/**
 	* @brief Set tolerance for custom force control.
 	*
 	* Specifies acceptable range for force/torque variations.
 	*
 	* @param tol Pointer to FTxyz structure specifying tolerances.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t set_cst_force_ctrl_tol(const FTxyz *tol, int robot_id = 0);

	/**
 	* @brief Get tolerance for custom force control.
 	*
 	* @param tol Pointer to FTxyz structure to receive tolerance values.
 	* @param robot_id ID of the robot. Default is 0.
 	*
 	* @return errno_t Indicate the status of operation. ERR_SUCC for success and other for failure.
 	*/
	errno_t get_cst_force_ctrl_tol(FTxyz *tol, int robot_id = 0);


	
	/**
	* @brief Get force control speed limit.
	*
	* @param vel Line speed limit, mm/s.
	* @param angularVel Angular speed limit, rad/s	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_compliant_speed_limit(double* vel, double* angularVel);

	/**
	* @brief Set force control speed limit.
	*
	* @param speed_limit Line speed limit, mm/s.
	* @param angular_speed_limit Angular speed limit, rad/s.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_compliant_speed_limit(double vel, double angularVel);

	/**
	* @brief Get the torque reference center.
	*
	* @param ref_point 0 represents the sensor center, 1 represents TCP
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torque_ref_point(int *refPoint);

	/**
	* @brief Set the torque reference center.
	*
	* @param ref_point 0 represents the sensor center, 1 represents TCP
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torque_ref_point(int refPoint);

	/**
	* @brief Get sensor sensitivity.
	*
	* @param threshold Torque or force threshold for each axis, ranging within [0, 1]. The larger the value, the less sensitive the sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_end_sensor_sensitivity_threshold(FTxyz *threshold);

	/**
	* @brief Set the sensor sensitivity.
	*
	* @param threshold for each axis, 0~1, the larger the value, the less sensitive the sensor
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_end_sensor_sensitivity_threshold(FTxyz threshold);
	/**
	* @brief Get the torque sensor data of specified type.
	*
	* @param type Type of the data to be retrieved: 1 for actTorque (compatibility interface),  2 for original readings,  3 for real data without gravity and bias.
	* @param data Pointer for the returned feedback data
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torque_sensor_data(int type, TorqSensorData* data);

	/**
	 * @brief Trigger sensor zeroing and blocking for 0.5 seconds
	 *
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	errno_t zero_end_sensor();

	/**
	@brief set force stop condition for next motion cmd
	*/
	errno_t set_force_stop_condition(ForceStopConditionList condition);

	/**
	* @brief Setup the communication for the torque sensor.
	*
	* @param type Commnunication type of the torque sensor, 0: type I/II/III/IV/V sensor, 1: type VI sensor.
	* @param ip_addr IP address of the torque sensor. Only for type I/III/IV sensor
	* @param port Port for the torque sensor. Only for type I/III/IV sensor
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torque_sensor_comm(const int type, const char *ip_addr, const int port);

	/**
	* @brief Get the communication settings of the torque sensor.
	*
	* @param type Pointer for the returned commnunication type.
	* @param ip_addr Pointer for the returned IP address.
	* @param port Pointer for the returned port.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torque_sensor_comm(int *type, char *ip_addr, int *port);

	/**
	* @brief Set the torque sensor low-pass filter parameter for force control.
	*
	* @param torque_sensor_filter Filter parameter, unit：Hz
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torque_sensor_filter(const float torque_sensor_filter);

	/**
	* @brief Get the filter parameter of force control.
	*
	* @param torque_sensor_filter Pointer for the returned filter parameter, unit：Hz
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torque_sensor_filter(float *torque_sensor_filter);

	/**
	* @brief Set soft force or torque limit for the torque sensor.
	*
	* @param torque_sensor_soft_limit Soft limit, fx/fy/fz is force limit in unit N and tx/ty/tz is torque limit unit：N*m.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torque_sensor_soft_limit(const FTxyz torque_sensor_soft_limit);

	/**
	* @brief Get current soft force or torque limit of the torque sensor.
	*
	* @param torque_sensor_soft_limit Pointer for the returned soft limits.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torque_sensor_soft_limit(FTxyz *torque_sensor_soft_limit);

	/**
	* @brief Set the type of torque sensor.
	* 
	* @param sensor_brand Type of the torque sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torsenosr_brand(int sensor_brand);

	/**
	* @brief Get the type of torque sensor.
	* 
	* @param sensor_brand  Pointer to the returned torque sensor type.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torsenosr_brand(int *sensor_brand);

	/**
	* @brief Set mode to turn on or turn off the torque sensor.
	* 
	* @param sensor_mode Mode of the torque sensor, 1 to turn on and 0 to turn off the torque sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torque_sensor_mode(int sensor_mode);

	/**
	*@brief check if torque sensor enabled or not
	*/
	errno_t get_torque_sensor_mode(int* sensor_mode);
///@}

///@name torque sensor payload
///@{
	/**
	* @brief Start to identify payload of the torque sensor.
	* 
	* @param joint_pos End joint position of the trajectory.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t start_torq_sensor_payload_identify(const JointValue *joint_pos);

	/**
	* @brief Get the status of torque sensor payload identification
	*
	* @param identify_status Pointer of the returned result. 0: done，1: no identification result ready to read，2: error.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torq_sensor_identify_staus(int *identify_status);

	/**
	* @brief Get identified payload of the torque sensor.
	*
	* @param payload Pointer to the returned identified payload in kg, mm.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torq_sensor_payload_identify_result(PayLoad *payload);

	/**
	* @brief Set the payload for the torque sensor in kg, mm.
	*
	* @param payload Payload of torque sensor.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torq_sensor_tool_payload(const PayLoad *payload);

	/**
	* @brief Get current payload of the torque sensor in kg, mm.
	*
	* @param payload Pointer to the returned payload.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torq_sensor_tool_payload(PayLoad *payload);
///@}

///@name tool drive
///@{


	/**
	* @brief Get coordinate system for tool drive.
	*
	* @param ftFrame Pointer for the return coordinate system, 0 for tcp and 1 for userframe.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tool_drive_frame(FTFrameType *ftFrame);

	/**
	* @brief Set the the coordinate system for tool drive.
	*
	* @param ftFrame Coordinate system option for tool drive, 0 for tcp and 1 for userframe.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_tool_drive_frame(FTFrameType ftFrame);

	/**
	* @brief Get the sensitivity setting for fusion drive.
	*
	* @param level Pointer for the returned sensitivity level,which ranges within [0,5] and 0 means off, the smaller the less sensitive.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_fusion_drive_sensitivity_level(int *level);

	/**
	* @brief Set the sensitivity level for the fusion drive.
	*
	* @param level Sensitivity level, which ranges within [0,5] and 0 means off, the smaller the less sensitive.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_fusion_drive_sensitivity_level(int level);

	/**
	* @brief Get the warning range of motion limit (singularity point and joint limit)
	*
	* @param range_level Range level, 1-5, the smaller the larger the warning range is.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_motion_limit_warning_range(int *warningRange);

	/**
	* @brief Set the warning range for motion limit, like singularity point and joint limit.
	*
	* @param range_level Range level, 1-5, the smaller the larger the warning range is.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_motion_limit_warning_range(int warningRange);

	/**
	* @brief Enable or disable the tool drive. customer may drag robot by the end flange or the tools mounted on it
	* 
	* @param handle Control handler of the cobot. 
	* @param enable_flag Option to indicate enable or disable: 1 to enable and 0 to disable
	* the admittance control.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t enable_tool_drive(const int enable_flag);

	/**
	* @brief Get the tool drive mode and state.
	*
	* @param enable Pointer for the returned value that indicating if the tool drive mode is enabled or not.
	* @param state Pointer for the returned value that indicating if current state of tool drive triggers singularity point, speed, joint limit warning.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tool_drive_state(int* enable, int *state);

	/**
	* @brief Set tool drive configuration.
	* 
	* @param cfg Configuration for the tool drive.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_tool_drive_config(ToolDriveConfig cfg);

	/**
	* @brief Get current configuration for tool drive.
	* 
	* @param cfg Pointer for the returned tool drive configuration.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tool_drive_config(RobotToolDriveCtrl* cfg);
///@}

///@name force control
///@{

	/**
	@brief enable or disable force control
	*/
	errno_t set_ft_ctrl_mode(BOOL enable);

	/**
	@brief get force control state
	*/
	errno_t get_ft_ctrl_mode(BOOL* enable);

	/**
	@brief set force control config
	*/
	errno_t set_ft_ctrl_config(AdmitCtrlType admit_ctrl_cfg);

	/**
	@brief get force control config
	*/
	errno_t get_ft_ctrl_config(RobotAdmitCtrl* cfg);


	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Set parameters for admittance control of the cobot.
	* 
	* @param axis ID of the axis to be controlled, axis with ID 0 to 5 corresponds to x, y, z, Rx, Ry, Rz.
	* @param opt  Enable flag. 0: disable, 1: enable.
	* @param ftUser  Force to move the cobot in maximum speed.
	* @param ftReboundFK  Ability to go back to initial position.
	* @param ftConstant Set to 0 when operate manually.
	* @param ftNnormalTrack Set to 0 when operate manually.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_admit_ctrl_config(int axis, int opt, double ftUser, double ftConstant, int ftNnormalTrack, double ftReboundFK);


	/**
	* @brief Enable or disable tool drive. It will only work when a torque sensor is equiped. THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	*
	* @param enable_flag Option to indicate enable or disable. 1 to enable and 0 to disable.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t enable_admittance_ctrl(const int enable_flag);

	

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Set compliance control type and sensor initialization status.
	*
	* @param sensor_compensation Whether to enable sensor compensation, 1 means enable is initialized, 0 means not initialized.
	* @param compliance_type 0 for not using any kind of compliance control method 1 for constant force compliance control, 2 for speed compliance control
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_compliant_type(int sensor_compensation, int compliance_type);

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Get compliance control type and sensor initialization status.
	*
	* @param sensor_compensation Whether to enable sensor compensation, 1 means enable is initialized, 0 means not initialized.
	* @param compliance_type 0 for not using any kind of compliance control method 1 for constant force compliance control, 2 for speed compliance control
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_compliant_type(int *sensor_compensation, int *compliance_type);

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Get admitrance control configurations.
	*
	* @param admit_ctrl_cfg Pointer for the returned admittance control configurations.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_admit_ctrl_config(RobotAdmitCtrl *admit_ctrl_cfg);
	

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Disabled force control of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t disable_force_control();

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Set the parameters for velocity complianance control.
	*
	* @param vel_cfg Prameters for velocity compliance control.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_vel_compliant_ctrl(const VelCom *vel_cfg);

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Set condition for compliance control.
	*
	* @param ft the max force, if over limit, the robot will stop movement
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_compliance_condition(const FTxyz *ft);


	/**
	* @brief Set the coordinate or frame for the force control.
	*
	* @param ftFrame Coordinate or frame option. 0: tool frame, 1:world frame.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_ft_ctrl_frame(FTFrameType ftFrame);

	/**
	* @brief Get the coordinate or frame of the force control.
	*
	* @param ftFrame Pointer for the returned frame option.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_ft_ctrl_frame(FTFrameType* ftFrame);

	/**
	* @brief Set the approaching speed limit of constant force compliant control.
	*
	* @param vel Line speed, mm/s.
	* @param angularVel Angular speed rad/s.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
    errno_t set_approach_speed_limit(double vel, double angularVel);

	/**
	* @brief Get the approaching speed limit of constant force compliant control.
	*
	* @param vel Pointer for the returned line speed, mm/s.
	* @param angularVel Pointer for the returned angular speed rad/s.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
    errno_t get_approach_speed_limit(double *vel, double *angularVel);

	/**
	* @brief Set the force/torque tolerance of constant force compliant control.
	*
	* @param force Force tolerance, N.
	* @param torque Torque tolerance, Nm.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
    errno_t set_ft_tolerance(double force, double torque);

	/**
	* @brief Get the force/torque tolerance of constant force compliant control.
	*
	* @param force Pointer for the returned force tolerance, N.
	* @param torque Pointer for the returned torque tolerance, Nm.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
    errno_t get_ft_tolerance(double *force, double *torque);

///@}

///@name FTP part
///@{
	/**
	* @brief Establish ftp connection with controller.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t init_ftp_client();

	/**
	* @brief Establish an encrypted ftp connection with the controller (requires app login and controller version support)
	*
	* @param password Robot login password
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t init_ftp_client_with_ssl(char* password);

	/**
	* @brief Disconnect ftp from controller
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t close_ftp_client();
	/**
	* @brief Download a file of the specified type and name from the controller to the local
	*
	* @param remote The absolute path to the file name inside the controller.
	* @param local The absolute path to the file name to be downloaded locally.
	* @param opt 1 single file 2 folder
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t download_file(char* local, char* remote, int opt);

	/**
	* @brief Upload a file of a specified type and name from the controller to the local
	*
	* @param remote Absolute path of the file name to be uploaded inside the controller.
	* @param local Absolute path to local file name.
	* @param opt 1 single file 2 folder
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t upload_file(char* local, char* remote, int opt);


	/**
	* @brief Delete a file of the specified type and name from the controller.
	*
	* @param remote Controller internal file name
	* @param opt 1 single file 2 folder
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t del_ftp_file(char* remote, int opt);

	/**
	* @brief Rename a file of the type and name specified by the controller.
	*
	* @param remote Original name of the controller's internal file name
	* @param des The target name to rename.
	* @param opt 1 single file 2 folder
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t rename_ftp_file(char* remote, char* des, int opt);

	/**
	* @brief Get the directory of the FTP service. 
	*
	* @param remotedir Pointer for the returned FIP directory. like "/track/" or "/program/"
	* @param type Type of the file. 0: file and folder, 1: single file, 2: folder
	* @param ret Returned structure of the directory in string format.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_ftp_dir(const char* remotedir, int type, char* ret);
///@}
	
///@name K1-SDK
///@{

	/**
	* @brief Compute the pose for a specified joint position using the current tool, mounting angle, and user coordinate settings
	* @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	* @param joint_pos Joint-space position
	* @param cartesian_pose Computed Cartesian pose result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t kine_forward(int robot_id, const JointValue *joint_pos, CartesianPose *cartesian_pose);	

	/**
	* @brief Compute the inverse kinematics for a specified pose using the current tool, mounting angle, and user coordinate settings
	* @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	* @param ref_pos Reference joint-space position used for IK computation
	* @param cartesian_pose Cartesian pose value
	* @param joint_pos Computed joint-space result on success
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t kine_inverse(int robot_id, const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos);

	/**
	* @brief Get the configured robot collision level
	* @param robot_id  
	* @param level Collision level, range 0-5: 0 disables collision detection, 1-5 map to 25N, 50N, 75N, 100N, and 125N thresholds
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_collision_level(int robot_id, int *level);

	/**
	* @brief Set the robot collision level
	* @param robot_id  
	* @param level Collision level, range 0-5: 0 disables collision detection, 1-5 map to 25N, 50N, 75N, 100N, and 125N thresholds
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_collision_level(int robot_id, const int level);

	/**
	* @brief Enter or exit drag mode
	* @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	* @param enable TRUE to enter drag mode, FALSE to exit
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t drag_mode_enable(int robot_id, BOOL enable);

	/**
	 * @brief Get the robot mounting pose
	 * @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	 * @param base_offset Transform from the robot base frame to the world frame, with orientation in ZYX Euler angles, units: mm and rad
	 * @return ERR_SUCC on success, otherwise failure
	 */
    errno_t robot_get_default_base(int robot_id, CartesianPose* base_offset);

	/**
	 * @brief Get the robot tool-frame offset
	 * @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	 * @param tool Robot tool-frame offset
	 * @return ERR_SUCC on success, otherwise failure
	 */
	errno_t robot_get_tool_offset(int robot_id, CartesianPose* offset);

	/**
	 * @brief Set the robot tool-frame offset
	 * @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	 * @param tool Robot tool-frame offset
	 * @return ERR_SUCC on success, otherwise failure
	 */
    errno_t robot_set_tool_offset(int robot_id, CartesianPose tool);

	/// @brief Check whether the robot is in an error state
	/// @param in_error 0 means normal state, 1 means error state
	/// @return 
    errno_t robot_is_in_error(int* in_error);
	
	
	/// @brief Check whether the robot currently exceeds the soft limit
    /// @param robot_id Accepts LEFT(0) or RIGHT(1)
    /// @param limit 0 means within limit, 1 means exceeded
    /// @return 
    errno_t robot_is_on_soft_limit(int robot_id, int* limit);


	/**
	 * @brief Enable or disable the joint control-loop parameters
	 * @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	 * @param joint_id Joint ID
	 * @param en Whether to enable the control-loop parameters
	 * @note After disabling controlloop, the parameters are reset to internal defaults
	 * @note Supported only on version 3.0.3_ZY1 and above
	 */
	errno_t enable_joint_controlloop(int robot_id, int joint_id, bool en);

	/**
	 * @brief Set joint control-loop parameters
	 * @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	 * @param joint_id Joint ID
	 * @param pos_kp Position-loop proportional gain, recommended 30, range 0-100. Larger values make the position loop stiffer; values that are too large may cause vibration
	 * @param vel_kp Velocity-loop proportional gain, recommended 50, range 0-200. Larger values make the velocity loop stiffer; values that are too large may cause vibration
	 * @param vel_ti Velocity-loop integral constant, recommended 2000, range 500-16777216. Larger values weaken the integral effect; values that are too small may cause overshoot or instability
	 * @note Effective only when controlloop is enabled. After disabling controlloop, the parameters are reset to internal defaults
	 * @note Supported only on version 3.0.3_ZY1 and above
	 */
	errno_t set_joint_controlloop(int robot_id, int joint_id, uint32_t pos_kp, uint32_t vel_kp, uint32_t vel_ti);


	/// @brief Set the cutoff frequency of the sensor data smoothing filter
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param sensor_id Sensor ID, valid range 1-199
	/// @param filter Cutoff frequency in Hz, must be positive
	/// @return
    errno_t robot_set_ftsensor_filter(int robot_id, int sensor_id, double filter);


	/// @brief Set the cutoff frequency of the sensor data smoothing filter (used by 3.3 controllers)
	/// @param sensor_id Sensor ID, valid range 1-199
	/// @param filter Cutoff frequency in Hz, must be positive
	/// @return
    errno_t robot_set_ftsensor_filter(int sensor_id, double filter);

	/// @brief Get the cutoff frequency of the sensor data smoothing filter
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param sensor_id Sensor ID, valid range 1-199
	/// @param filter Cutoff frequency in Hz, must be positive
	/// @return
    errno_t robot_get_ftsensor_filter(int robot_id, int sensor_id, double* filter);

	/// @brief Get the cutoff frequency of the sensor data smoothing filter (used by 3.3 controllers)
	/// @param sensor_id Sensor ID, valid range 1-199
	/// @param filter Cutoff frequency in Hz, must be positive
	/// @return
    errno_t robot_get_ftsensor_filter(int sensor_id, double* filter);

	/// @brief Get the force-control mode status; note that after calling disable_force_control, force control is still considered enabled until the current motion finishes
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param force_control_stat 0 means disabled, 1 means enabled
	/// @return
    errno_t robot_get_force_control_stat(int robot_id, int* force_control_stat);

	/// @brief Get sensor status
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param sensor_id Sensor ID, valid range 1-199
	/// @param status Normal(1) or error(-1)
	/// @param errcode No data(1), data error(2), overload(4)
	/// @param ft_original Raw sensor readings in the sensor's own coordinate frame
	/// @param ft_actual Readings in the flange frame after payload and zero-offset compensation
	/// @return 
    errno_t robot_get_ftsensor_stat(int robot_id, int sensor_id, int* status, int* errcode, double* ft_original, double* ft_actual);	

	/// @brief Get sensor status (used by 3.3 controllers)
	/// @param sensor_id Sensor ID, valid range 1-199
	/// @param status Normal(1) or error(-1)
	/// @param errcode No data(1), data error(2), overload(4)
	/// @param ft_original Raw sensor readings in the sensor's own coordinate frame
	/// @param ft_actual Readings in the flange frame after payload and zero-offset compensation
	/// @return 
    errno_t robot_get_ftsensor_stat(int sensor_id, int* status, int* errcode, double* ft_original, double* ft_actual);	

	/// @brief Set the sensor payload; note that the sensor payload may differ from the robot payload and must be configured separately
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param sensor_id Sensor ID, valid range 1-199 (embedded sensors do not support changing the payload ID)
	/// @param payload_id Payload ID
	/// @return
    errno_t robot_set_ftsensor_payload(int robot_id, int sensor_id, int payload_id);

	/// @brief Set the sensor payload; note that the sensor payload may differ from the robot payload and must be configured separately (used by 3.3 controllers)
	/// @param sensor_id Sensor ID, valid range 1-199 (embedded sensors do not support changing the payload ID)
	/// @param payload_id Payload ID
	/// @return
    errno_t robot_set_ftsensor_payload(int sensor_id, int payload_id);

	/// @brief Get sensor payload settings; note that the sensor payload may differ from the robot payload and must be configured separately
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param sensor_id Sensor ID, valid range 1-199
	/// @param payload Payload data, units: kg and mm
	/// @return
    errno_t robot_get_ftsensor_payload(int robot_id, int sensor_id, PayLoad* payload);


	/// @brief Get sensor payload settings; note that the sensor payload may differ from the robot payload and must be configured separately (used by 3.3 controllers)
	/// @param sensor_id Sensor ID, valid range 1-199
	/// @param payload Payload data, units: kg and mm
	/// @return
    errno_t robot_get_ftsensor_payload(int sensor_id, PayLoad* payload);

	/// @brief Sensor zeroing API. Zeroing takes about 1 second, and force control cannot be enabled during this period, so waiting 1 second after the call is recommended.
	/// It is generally recommended to zero the sensor before enabling force control. During zeroing, the sensor should not be subject to any external force other than payload gravity.
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param sensor_id Sensor ID, valid range 1-199
	/// @return
    errno_t robot_zero_ftsensor(int robot_id, int sensor_id);


	/// @brief Sensor zeroing API. Zeroing takes about 1 second, and force control cannot be enabled during this period, so waiting 1 second after the call is recommended. (used by 3.3 controllers)
	/// It is generally recommended to zero the sensor before enabling force control. During zeroing, the sensor should not be subject to any external force other than payload gravity.
	/// @param sensor_id Sensor ID, valid range 1-199
	/// @return
    errno_t robot_zero_ftsensor(int sensor_id);

	/// @brief Enable constant-force compliance control. This mode can be used together with linear motion, joint motion, and Cartesian servo modes, but not with drag mode or manual JOG teaching.
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @return 
    errno_t robot_enable_force_control(int robot_id);

	/// @brief Disable constant-force compliance control. Note that disabling force control makes it ineffective immediately, but does not leave force-control mode right away.
	/// Force-control mode exits only after the robot completes the current motion (or leaves servo mode), so it cannot be re-enabled during that interval.
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @return 
    errno_t robot_disable_force_control(int robot_id);


	/// @brief Set constant-force compliance parameters
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param axis Axis to configure, valid range 0-5 corresponding to x, y, z, rx, ry, rz
	/// @param enable Whether force control is enabled, accepts off(0) or on(1)
	/// @param ftUser Force-control stiffness, must be positive and should not be 0. Values above 10 are generally recommended for x/y/z and above 1 for mx/my/mz
	/// @param ftReboundFK Rebound elasticity coefficient, must be positive and may be set to 0
	/// @param ftConstant Target force; do not set a target force beyond the robot's supported range
	/// @return 
    errno_t robot_set_cst_ftconfig(int robot_id, int axis, int enable, double ftUser, double ftReboundFK, double ftConstant);

	/// @brief Get constant-force compliance parameters
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param config Constant-force compliance parameters
    errno_t robot_get_cst_ftconfig(int robot_id, RobotAdmitCtrl* config);


	/// @brief Synchronous multi-robot joint-motion command
	/// @param robot_id Robot ID, accepts LEFT(0), RIGHT(1), or DUAL(-1)
	/// @param move_mode ABS (absolute motion) or INCR (relative motion)
	/// @param is_block TRUE (blocking motion) or FALSE (non-blocking motion)
	/// @param joint_pos Position commands for both robots
	/// @param vel Velocity commands for both robots
	/// @param acc Acceleration commands for both robots
	/// @param tol Allowed transition error for both robot trajectories, range >= 0
	/// @return 
    errno_t robot_run_multi_movj(int robot_id, const MoveMode *move_mode, BOOL is_block, const JointValue *joint_pos, const double* vel, const double* acc, const double* tol = nullptr);

	/// @brief Synchronous multi-robot Cartesian-motion command
	/// @param robot_id Robot ID, accepts LEFT(0), RIGHT(1), or DUAL(-1)
	/// @param move_mode ABS (absolute motion) or INCR (relative motion)
	/// @param is_block TRUE (blocking motion) or FALSE (non-blocking motion)
	/// @param end_pos Position commands for both robots
	/// @param vel Velocity commands for both robots
	/// @param acc Acceleration commands for both robots
	/// @param tol Allowed transition error for both robot trajectories, range >= 0
	/// @return 
    errno_t robot_run_multi_movl(int robot_id, const MoveMode *move_mode, BOOL is_block, const CartesianPose* end_pos, const double* vel, const double* acc, const double* tol = nullptr);


    /// @brief Get the DH parameters of both robots
    /// @param dh_param DH parameters, 2-dimensional
    /// @return 
    errno_t robot_get_multi_robot_dh(DHParam *dh_param);

    /// @brief Get whether both robots are in position
    /// @param inpos 1 means in position, 0 means not in position; 2-dimensional
    /// @return 
    errno_t robot_is_inpos(int* inpos);

    /// @brief Set the end-effector tool payload
    /// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
    /// @param payload Robot payload parameters, 2-dimensional
    /// @return 
    errno_t robot_set_tool_payload(int robot_id, const PayLoad* payload);

	/// @brief Get the end-effector tool payload
    /// @param payload Robot payload parameters, 2-dimensional
    /// @return 
    errno_t robot_get_tool_payload(PayLoad* payload);

	/// @brief Set sensor sensitivity
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param sensor_id Sensor ID, valid range 1-20
	/// @param deadzone_percent Percentage of the default maximum dead zone used as the sensor sensitivity dead zone, range 0-1
	/// @return 
    errno_t robot_set_ftsensor_deadzone(int robot_id, int sensor_id, const double* deadzone_percent);

	/// @brief Set sensor sensitivity (used by 3.3 controllers)
	/// @param sensor_id Sensor ID, valid range 1-20
	/// @param deadzone_percent Percentage of the default maximum dead zone used as the sensor sensitivity dead zone, range 0-1
	/// @return 
    errno_t robot_set_ftsensor_deadzone(int sensor_id, const double* deadzone_percent);


	/// @brief Get sensor sensitivity
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param sensor_id Sensor ID, valid range 1-20
	/// @param deadzone_percent Percentage of the default maximum dead zone used as the sensor sensitivity dead zone, range 0-1
	/// @return 
    errno_t robot_get_ftsensor_deadzone(int robot_id, int sensor_id, double* deadzone_percent);

	/// @brief Get sensor sensitivity (used by 3.3 controllers)
	/// @param sensor_id Sensor ID, valid range 1-20
	/// @param deadzone_percent Percentage of the default maximum dead zone used as the sensor sensitivity dead zone, range 0-1
	/// @return 
    errno_t robot_get_ftsensor_deadzone(int sensor_id, double* deadzone_percent);

	/// @brief Get the force-control coordinate frame
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param ftframe Force-control coordinate frame, accepts tool(0) or world(1)
	/// @return 
    errno_t robot_get_cst_ftframe(int robot_id, int* ftframe);

	/// @brief Set the force-control coordinate frame
	/// @param robot_id Robot ID, accepts LEFT(0) or RIGHT(1)
	/// @param ftframe Force-control coordinate frame, accepts tool(0) or world(1)
	/// @return 
    errno_t robot_set_cst_ftframe(int robot_id, int ftframe);


	/**
	* @brief Enable or disable full-kinematics compensation for the 7th axis; the setting takes effect for the robot group together
	* @param flag 1 enables compensation, 0 disables it
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_full_dh_flag(int flag);

	/**
	* @brief Set the gravity direction relative to the robot base; the setting takes effect for the robot group together
	* @param rpy RPY rotation angles; T = rotx(rpy[0]) * roty(rpy[1]) * rotz(rpy[2])
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_gravity_direction(const double* rpy);

	/**
	* @brief Read the gravity direction relative to the robot base
	* @param rpy RPY rotation angles; T = rotx(rpy[0]) * roty(rpy[1]) * rotz(rpy[2])
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_gravity_direction(double* rpy);

	/**
	 * @brief Get the FREE-button state on the robot end effector
	 * @param keys FREE-button states, where 0 means not pressed and 1 means pressed, one for each robot
	 * @return ERR_SUCC on success, otherwise failure
	 */
	errno_t edg_stat_free_key(bool keys[2]);

	/**
	 * @brief Trigger the internal receive event in a periodic task and refresh the cache
	 * @param next Timestamp of the next state sample
	 * @return ERR_SUCC on success, otherwise failure
	 * @deprecated This API will be removed soon and does not need to be called inside periodic tasks
	 */
	errno_t edg_recv(struct timespec *next = nullptr);

	/**
	 * @brief Trigger the internal send event in a periodic task. Use together with edg_servo_p and edg_servo_j; commands are sent only after calling edg_send.
	 * @param cmd_index Command index used to identify command uniqueness; if nullptr, an internal auto-incremented index is used
	 * @return ERR_SUCC on success, otherwise failure
	 */
	errno_t edg_send(const uint32_t *cmd_index = nullptr);

	/**
	 * @brief Send joint-position commands in real time at a fixed period, in radians. For example, the CAN arm uses 8 ms.
	 * @param robot_index Robot index, where 0 is the left arm and 1 is the right arm
	 * @param joint_pos Joint-position command, in radians
	 * @param move_mode Motion mode, ABS for absolute motion and INCR for relative motion
	 * @param step_num Number of steps to send the command, default is 1. step_num=1 is recommended so state is refreshed immediately after sending.
	 * @return ERR_SUCC on success, otherwise failure
	 */
	errno_t edg_servo_j(unsigned char robot_index, const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num=1);

	/**
	 * @brief Send Cartesian position commands at a fixed period, in mm and rad
	 * @param robot_index Robot index, where 0 is the left arm and 1 is the right arm
	 * @param cartesian_pose Cartesian position command, in mm and rad
	 * @param move_mode Motion mode, ABS for absolute motion and INCR for relative motion
	 * @param step_num Number of steps to send the command, default is 1. For example, if the call period cannot be guaranteed at 8 ms, a larger value such as 2 allows a 16 ms call period.
	 * 				This is mainly used when client-side real-time performance is poor, larger jitter must be tolerated, or the processing cycle is longer.
	 * @return ERR_SUCC on success, otherwise failure
	 */
	errno_t edg_servo_p(unsigned char robot_index, const CartesianPose *cartesian_pose, MoveMode move_mode, unsigned int step_num=1);


	/**
	 * @brief Get robot state. Note that this API returns cached data refreshed roughly every 8 ms.
	 * @param robot_index Robot index, where 0 is the left arm and 1 is the right arm
	 * @param joint_pos Joint positions, in radians
	 * @param cartesian_pose Cartesian position, in mm and rad
	 * @param sensor_torque Sensor torque, in Nm
	 * @param joint_torque Joint torque, in Nm
	 * @return ERR_SUCC on success, otherwise failure
	 */
	errno_t edg_get_stat(unsigned char robot_index, JointValue *joint_pos, CartesianPose *cartesian_pose, CartesianPose *sensor_torque=nullptr, JointValue *joint_torque = nullptr);

	/**
	* @brief Set TIO voltage parameters for the cobot robot. Only effective for TIO hardware version 3.
	* @param robot_id Robot index, where 0 is the left arm and 1 is the right arm
	* @param vout_enable Voltage-output enable option, 0: disabled, 1: enabled
	* @param vout_vol Output-voltage option, 0: 24V
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_tio_vout_param(int robot_id, int vout_enable, int vout_vol);

	/**
	* @brief Get TIO voltage parameters for the cobot robot. Only effective for TIO hardware version 3.
	* @param robot_id Robot index, where 0 is the left arm and 1 is the right arm
	* @param vout_enable Pointer returning the voltage-output enable option, 0: disabled, 1: enabled
	* @param vout_vol Pointer returning the output-voltage option, 0: 24V, 1: 12V
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_tio_vout_param(int robot_id, int* vout_enable, int* vout_vol);

	/**
	* @brief Set TIO mode
	* @param robot_id Robot index, where 0 is the left arm and 1 is the right arm
	* @param pin_type TIO type: 0 for DI pins, 1 for DO pins, 2 for AI pins
	* @param pin_mode TIO mode:
	*       DI pins:
 	*          0: 0x00, DI2 is NPN and DI1 is NPN
	*          1: 0x01, DI2 is NPN and DI1 is PNP
 	*          2: 0x10, DI2 is PNP and DI1 is NPN
 	*          3: 0x11, DI2 is PNP and DI1 is PNP
 	*        DO pins:
 	*          Lower 8 bits store data, upper 4 bits configure DO2, lower 4 bits configure DO1
	*          0x0 DO is NPN output
 	*          0x1 DO is PNP output
 	*          0x2 DO is push-pull output
 	*          0xF RS485H interface
 	*        AI pins:
 	*          0: analog-input function enabled, RS485L disabled
 	*          1: RS485L interface enabled, analog-input function disabled
 	*
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_tio_pin_mode(int robot_id, int pin_type, int pin_mode);

	/**
	* @brief Get the mode of the specified TIO pin type
	* @param robot_id Robot index, where 0 is the left arm and 1 is the right arm
	* @param pin_type TIO type: 0 for DI pins, 1 for DO pins, 2 for AI pins
	* @param pin_mode TIO mode:
	*       DI pins:
 	*          0: 0x00, DI2 is NPN and DI1 is NPN
	*          1: 0x01, DI2 is NPN and DI1 is PNP
 	*          2: 0x10, DI2 is PNP and DI1 is NPN
 	*          3: 0x11, DI2 is PNP and DI1 is PNP
 	*        DO pins:
 	*          Lower 8 bits store data, upper 4 bits configure DO2, lower 4 bits configure DO1
	*          0x0 DO is NPN output
 	*          0x1 DO is PNP output
 	*          0x2 DO is push-pull output
 	*          0xF RS485H interface
 	*        AI pins:
 	*          0: analog-input function enabled, RS485L disabled
 	*          1: RS485L interface enabled, analog-input function disabled
 	*
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_tio_pin_mode(int robot_id, int pin_type, int* pin_mode);

	/**
	* @brief Configure communication for the specified RS485 channel
	* @param robot_id Robot index, where 0 is the left arm and 1 is the right arm
	* @param ModRtuComm When the channel mode is Modbus RTU, an additional Modbus slave ID must be provided
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_rs485_chn_comm(int robot_id, ModRtuComm mod_rtu_com);

	/**
	* @brief Get RS485 communication settings
	* @param robot_id Robot index, where 0 is the left arm and 1 is the right arm
	* @param mod_rtu_com Pointer returning the communication settings of the specified channel
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_rs485_chn_comm(int robot_id, int chn_id,ModRtuComm* mod_rtu_com);

	/**
	* @brief Set the mode of the specified RS485 channel
	* @param robot_id Robot index, where 0 is the left arm and 1 is the right arm
	* @param chn_id Channel ID, where 0 is RS485H (channel 1) and 1 is RS485L (channel 2)
	* @param chn_mode Channel mode, where 0 is Modbus RTU, 1 is raw RS485, and 2 is torque sensor
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_rs485_chn_mode(int robot_id, int chn_id, int chn_mode);

	/**
	* @brief Get the mode of the specified RS485 channel
	* @param robot_id Robot index, where 0 is the left arm and 1 is the right arm
	* @param chn_id Channel ID, where 0 is RS485H (channel 1) and 1 is RS485L (channel 2)
	* @param chn_mode Pointer receiving the returned mode: 0 for Modbus RTU, 1 for raw RS485, 2 for torque sensor
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_rs485_chn_mode(int robot_id, int chn_id, int* chn_mode);

	/**
	* @brief Send a command on the specified RS485 channel
	* @param robot_id Robot index, where 0 is the left arm and 1 is the right arm
	* @param chn_id RS485 channel ID in TIO
	* @param data Command data
	* @param buffsize Length in bytes of the data pointed to by data, i.e. the number of bytes to send
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t send_tio_rs_command(int robot_id, int chn_id, uint8_t* data,int buffsize);
		
///@}


	~JAKAZuRobot();

private:
	class BIFClass;
	BIFClass *ptr;
	
	JAKARobotInterface* robot;
	bool login = false;
	std::string ip;

	const double PI = 3.1415926; 
	const int Tool_UserFrame_Name_Length_MAX = 128;

	inline double Rad2Degree(double in){
		return in * 180 / PI;
	}
	inline JointValue Rad2Degree(JointValue in){
		JointValue out;
		for (int j  = 0; j < 6; ++j) {
			out.jVal[j] = in.jVal[j] * 180 / PI;
		}
		return out;
	}
	inline CartesianPose Rad2Degree(CartesianPose in){
		CartesianPose out = in;
		out.rpy.rx = in.rpy.rx * 180 / PI;
		out.rpy.ry = in.rpy.ry * 180 / PI;
		out.rpy.rz = in.rpy.rz * 180 / PI;
		return out;
	}
	inline double Degree2Rad(double in){
		return in * PI /180;
	}
	inline Rpy Degree2Rad(Rpy in){
		Rpy out;
		out.rx = in.rx / 180 * PI;
		out.ry = in.ry / 180 * PI;
		out.rz = in.rz / 180 * PI;
		return out;
	}
	inline JointValue Degree2Rad(JointValue in){
		JointValue out;
		for (int j  = 0; j < 6; ++j) {
			out.jVal[j] = in.jVal[j] * PI / 180;
		}
		return out;
	}
	inline CartesianPose Degree2Rad(CartesianPose in){
		CartesianPose out = in;
		out.rpy.rx = in.rpy.rx * PI / 180;
		out.rpy.ry = in.rpy.ry * PI / 180;
		out.rpy.rz = in.rpy.rz * PI / 180;
		return out;
	}
	inline DHParam Degree2Rad(DHParam in){
		DHParam out = in;
		for (int i = 0; i<6; ++i) {
            out.alpha[i] = in.alpha[i]* PI/180;
            out.a[i] = in.a[i];
            out.d[i] = in.d[i];
            out.joint_homeoff[i] = in.joint_homeoff[i]* PI/180;
        }
		return out;
	}
};


#undef DLLEXPORT_API
#endif
