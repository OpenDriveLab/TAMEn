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
 * File: jakaAPI.h
 * @author star@jaka
 * @date Nov-30-2021 
 *
**************************************************************************/


#ifndef _JAKA_API_
#define _JAKA_API_

#include "jkerr.h"
#include "jktypes.h"

#if defined(_WIN32) || defined(WIN32)
/**
 * for low version of Visual Studio, we need to define __cpluscplus manually
 */
#ifdef __cpluscplus

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

#ifdef __cpluscplus
extern "C"
{
#endif // __cpluscplus

///@name general part
///@{
	/**
	* @brief Create a control handler for the cobot with specified IP address.
	*
	* @param ip  IP address of the cobot controller.
	* @param handle Pointer to the returned cobot control handler.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t create_handler(const char *ip, JKHD *handle, BOOL use_grpc);

	/**
	* @brief Destroy the handler of the cobot. The connection with the cobot will be then terminated.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t destory_handler(const JKHD *handle);

	/**
	* @brief Power on the cobot.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t power_on(const JKHD *handle);

	/**
	* @brief Power off the cobot.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t power_off(const JKHD *handle);

	/**
	* @brief Shut down the control cabinet. The cobot must be disabled and powered off before executing this command.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t shut_down(const JKHD *handle);

	/**
	* @brief Enable the cobot. The cobot must be powered on before executing this command.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t enable_robot(const JKHD *handle);

	/**
	* @brief Disable the cobot.The cobot must not be running a program before executing this command.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t disable_robot(const JKHD *handle);
	
	/**
	* @brief Get current status of the cobot(A large struct contains as more data as possible).
	* @deprecated please check each inner data using corresponding interfaces instead of this one. 
	* @warning please note that config of OptionalInfoConfig.ini may affect data from port 10004 and data in RobotStatus without error
	* if you're not familiar with this, please keep OptionalInfoConfig.ini no changed as defalut setting. 
	*
	* @param handle Control handler of the cobot. 
	* @param status Pointer for the returned cobot status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_robot_status(const JKHD *handle, RobotStatus *status);

	/**
	* @brief Get current status of the cobot(A small struct only contains error/poweron/enable info).
	*
	* @param handle  Control handler of the cobot. 
	* @param status Pointer for the returned cobot status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_robot_status_simple(const JKHD *handle, RobotStatus_simple *status);
	
	/**
	* @brief Get current status of the cobot
	* @deprecated please use "get_robot_status_simple" instead
	*
	* @param handle  Control handler of the cobot. 
	* @param state Pointer for the returned cobot status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_robot_state(const JKHD *handle, RobotState *state);
	
	/**
	* @brief Get the Denavit–Hartenberg parameters of the cobot.
	* 
	* @param handle Control handler of the cobot.
	* @param dhParam Pointer of a varible to save the returned Denavit–Hartenberg parameters.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_dh_param(const JKHD *handle, DHParam *dhParam);

	/**
	* @brief Set installation (or mounting) angle of the cobot.
	* 
	* @param handle Control handler of the cobot.
    * @param angleX Rotation angle around the X-axis
    * @param angleZ Rotation angle around the Z-axis
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_installation_angle(const JKHD* handle, double angleX, double angleZ);

	/**
	* @brief Get installation (or mounting) angle of the cobot.
	* 
	* @param handle Control handler of the cobot.
	* @param quat Pointer for the returned result in quaternion.
	* @param appang Pointer for the returned result in RPY.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_installation_angle(const JKHD* handle, Quaternion* quat, Rpy* appang);

	/**
	* @brief Registor Callback funtion used when controller raise error	
	* @deprecated Only useful before SDK 2.1.12, check return code of each interface instead of using this
	*
	* @param handle  Control handler of the cobot. 
	* @param func Callback function
	*
	* @param error_code Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_error_handler(const JKHD *handle, CallBackFuncType func);

///@}

///@name motion part
///@{	
	/**
	 * @brief set global motion planner
	 */
	DLLEXPORT_API errno_t set_motion_planner(const JKHD* handle, MotionPlannerType type);

	/**
	* @brief Set timeout of motion block
	* @deprecated  Used only in SDK version before v2.1.12
	* 
	* @param handle  Control handler of the cobot. 
	* @param seconds Timeout, unit: second
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_block_wait_timeout(const JKHD* handle, float seconds);

	/**
	* @brief Set time interval of getting data via port 10004
	* @deprecated  Used only in SDK version before v2.1.12
	*
	* @param handle  Control handler of the cobot. 
	* @param millisecond Time interval, unit: millisecond
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_status_data_update_time_interval(const JKHD *handle, float millisecond);

	/**
	* @brief Jog a single joint or axis of the cobot to move in specified mode
	*
	* @param handle Control handler of the cobot.
    * @param aj_num 0-based axis or joint number, indicating joint number 0-5 in joint space, and x, y, z, rx, ry, rz in Cartesian space
	* @param move_mode Indicate the mode of the jog movement.
	* @param coord_type Indicated the coordinate of the jog movement.
	* @param vel_cmd Command velocity of the jog movement, unit rad/s for jog in joint space and mm/s for jog in Cartesian space.
	* @param pos_cmd Command target position, unit rad for jog in joint space and mm	for jog in Cartesian space.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t jog(const JKHD *handle, int aj_num, MoveMode move_mode, CoordType coord_type, double vel_cmd, double pos_cmd);

	/**
	* @brief Stop the ongoing jog movement.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t jog_stop(const JKHD *handle, int num);

	/**
	* @brief Move the cobot in joint space, without consideration of the path of TCP in Cartesian space.
	*
	* @param handle Control handler of the cobot.
	* @param joint_pos Target joint position of the joint movement.
	* @param move_mode Indicate the mode of the joint movement,which can be an incremental or absolute movement.
	* @param is_block Option to block the execution. The function call will not return until motion is done if the option is TRUE.
	* @param speed Command speed of the joint movement, in unit rad/s.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t joint_move(const JKHD *handle, const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed);

	/**
	* @brief Move the cobot in joint space, without consideration of the path of TCP in Cartesian space. Compared with 
	* joint_move command, this function provide more options for user to control the movement.
	*
	* @param handle Control handler of the cobot. 
	* @param joint_pos Target joint position of the joint movement.
	* @param move_mode Indicate the mode of the joint movement,which can be an incremental or absolute movement.
	* @param is_block Option to block the execution. The function call will not return until motion is done if the option is TRUE.
	* @param speed Command speed of the joint movement, in unit rad/s.
	* @param acc Command acceleration of the joint movement. in unit rad/s^2
	* @param tol Tolerance of the movement, usually considered as the radius of the blending arc with next movement, unit in mm.
	* @param option_cond Optional conditions for the movement, set NULL if not used.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t joint_move_extend(const JKHD *handle, const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed, double acc, double tol, const OptionalCond *option_cond);

	/**
	* @brief Move the cobot in Cartesian space, the TCP will move linearly.
	*
	* @param handle Control handler of the cobot. 
	* @param end_pos Target position of the movement.
	* @param move_mode Indicate the mode of the movement,which can be an incremental or absolute movement.
	* @param is_block Option to block the execution. The function call will not return until motion is done if the option is TRUE.
	* @param speed Command speed of the linear movement, in unit mm/s.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t linear_move(const JKHD *handle, const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed);

	/**
	* @brief Move the cobot in Cartesian space, the TCP will move linearly. More options will be provided compared with command linear_move.
	* 
	* @param handle Control handler of the cobot. 
	* @param end_pos Target Cartesian position of the movement.
	* @param move_mode Indicate the mode of the movement,which can be an incremental or absolute movement.
	* @param is_block Option to block the execution. The function call will not return until motion is done if the option is TRUE.
	* @param speed Command speed of the linear movement, in unit mm/s.
	* @param acc linear acceleration command for the movement, in unit mm/s^2
	* @param tol Tolerance of the movement, usually considered as the radius of the blending arc with next movement, unit in mm.
	* @param option_cond Optional conditions for the movement, set NULL if not used.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t linear_move_extend(const JKHD *handle, const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond);
	
	/**
	* @brief Move the cobot in Cartesian space and the TCP will move linearly. More options will be provided compared with command linear_move,
	* and user can specify the acceleration and velocity for orientation
	*
	* @param end_pos Target Cartesian position of the movement.
	* @param move_mode Indicate the mode of the movement,which can be an incremental or absolute movement.
	* @param is_block Option to block the execution. The function call will not return until motion is done if the option is TRUE.
	* @param speed Command speed of the linear movement, in unit mm/s.
	* @param acc linear acceleration command for the movement, in unit mm/s^2
	* @param tol Tolerance of the movement, usually considered as the radius of the blending arc with next movement, unit in mm.
	* @param option_cond Optional conditions for the movement, set NULL if not used.
	* @param ori_vel Velocity limit of the orientation change, in unit rad/s
	* @param ori_acc Accleration limit of orientation change, in unit rad/s^2	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t linear_move_extend_ori(const JKHD *handle, const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond, double ori_vel, double ori_acc);

	/**
	* @brief Move the cobot in Cartesan space and the TCP will move along the definded arc.
	* 
	* @param end_pos Target Cartesian position of the circular movement.
	* @param mid_pos Middle point of the circular path, which can be used to defined the move direction along the circle.
	* @param move_mode Indicate the mode of the movement,which can be an incremental or absolute movement.
	* @param is_block Option to block the execution. The function call will not return until motion is done if the option is TRUE.
	* @param speed Command speed of the linear movement, in unit mm/s.
	* @param acc linear acceleration command for the movement, in unit mm/s^2
	* @param tol Tolerance of the movement, usually considered as the radius of the blending arc with next movement, unit in mm.
	* @param option_cond Optional conditions for the movement, set NULL if not used.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t circular_move(const JKHD *handle, const CartesianPose *end_pos, const CartesianPose *mid_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond);

	/**
	* @brief Move the cobot in Cartesan space and the TCP will move along the definded arc.
	* 
	* @param end_pos Target Cartesian position of the circular movement.
	* @param mid_pos Middle point of the circular path, which can be used to defined the move direction along the circle.
	* @param move_mode Indicate the mode of the movement,which can be an incremental or absolute movement.
	* @param is_block Option to block the execution. The function call will not return until motion is done if the option is TRUE.
	* @param speed Command speed of the linear movement, in unit mm/s.
	* @param acc linear acceleration command for the movement, in unit mm/s^2
	* @param tol Tolerance of the movement, usually considered as the radius of the blending arc with next movement, unit in mm.
	* @param option_cond Optional conditions for the movement, set NULL if not used.
	* @param circle_cnt Circle count of the movement, 0 by default.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t circular_move_extend(const JKHD *handle, const CartesianPose *end_pos, const CartesianPose *mid_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond, double circle_cnt);

	/**
	* @brief Move the cobot in Cartesan space and the TCP will move along the definded arc.
	* 
	* @param end_pos Target Cartesian position of the circular movement.
	* @param mid_pos Middle point of the circular path, which can be used to defined the move direction along the circle.
	* @param move_mode Indicate the mode of the movement,which can be an incremental or absolute movement.
	* @param is_block Option to block the execution. The function call will not return until motion is done if the option is TRUE.
	* @param speed Command speed of the linear movement, in unit mm/s.
	* @param acc linear acceleration command for the movement, in unit mm/s^2
	* @param tol Tolerance of the movement, usually considered as the radius of the blending arc with next movement, unit in mm.
	* @param option_cond Optional conditions for the movement, set NULL if not used.
	* @param circle_cnt Circle count of the movement, 0 by default.
	* @param circle_mode Indicate the mode of the movement:
	* 0：Minimum orientation change: Robot moves by minimum orientation changes according to start point orientation, midpoint orientation, and end point orientation
	* 1：Large orientation change: Robot moves by large orientation changes according to start point orientation, midpoint orientation, and end point orientation.
	* 2：Midpoint orientation adaption: The MoveC trajectory is determined by start point orientation, midpoint position, and end point orientation; while the MoveC orientation of the robot is determined by midpoint orientation. Robot moves by the MoveC in the way of minimum or large attitude change
	* 3：Fixed orientation angle: When the start point orientation is different from end point orientation, the robot moves in accordance with the start point orientation and its TCP always points to the center of the circle according to the start point orientation, midpoint position, and end point position, regardless of the midpoint orientation, and end point orientation.
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t circular_move_extend_mode(const JKHD *handle, const CartesianPose *end_pos, const CartesianPose *mid_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond, double circle_cnt, int circle_mode);

	/**
	* @brief Set velocity rate (or velociry override).
	*
	* @param handle Control handler of the cobot. 
	* @param rapid_rate Value of the velociry rate, range from [0,1].
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_rapidrate(const JKHD *handle, double rapid_rate);

	/**
	* @brief Get velocity rate (or velociry override).
	*
	* @param handle Control handler of the cobot. 
	* @param rapid_rate Pointer for the returned current velocity rate.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_rapidrate(const JKHD *handle, double *rapid_rate);

	/**
	* @brief Define the data of tool with specified ID.
	*
	* @param handle Control handler of the cobot.
	* @param id ID of the tool to be defined.
	* @param tcp Data of the tool to be set.
	* @param name Alias name of the tool to be set.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_tool_data(const JKHD *handle, int id, const CartesianPose *tcp, const char *name);

	/**
	* @brief Switch to the tool with specified ID.
	*
	* @param handle Control handler of the cobot. 
	* @param id ID of the tool.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_tool_id(const JKHD *handle, const int id);

	/**
	* @brief Get ID of the tool currently used.
	*
	* @param handle Control handler of the cobot. 
	* @param id Pointer for the returned current tool ID.		
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_tool_id(const JKHD *handle, int *id);

	/**
	* @brief Get definition data of the tool with specified ID.
	*
	* @param handle Control handler of the cobot. 
	* @param id ID of the tool.
	* @param tcp Pointer for the returned tool data	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_tool_data(const JKHD *handle, int id, CartesianPose *tcp);

	/**
	* @brief Define the data of user frame with specified ID.
	*
	* @param handle Control handler of the cobot.
	* @param id ID of the user frame.
	* @param user_frame Data of the user frame to be set.
	* @param name Alias	name of the user frame.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_user_frame_data(const JKHD *handle, int id, const CartesianPose *user_frame, const char *name);

	/**
	* @brief Switch to the user frame with specified ID.
	*
	* @param handle Control handler of the cobot. 
	* @param id ID of the user frame.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_user_frame_id(const JKHD *handle, const int id);

	/**
	* @brief Get ID of the user frame currently used.
	*
	* @param handle Control handler of the cobot. 
	* @param id Pointer for the returned current user frame ID.		
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_user_frame_id(const JKHD *handle, int *id);

	/**
	* @brief Get definition data of the user frame with specified ID.
	*
	* @param handle Control handler of the cobot. 
	* @param id ID of the user frame.
	* @param tcp Pointer for the returned user frame data	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_user_frame_data(const JKHD *handle, int id, CartesianPose *user_frame);

	/**
	* @brief Set payload for the cobot.
	*
	* @param handle Control handler of the cobot. 
	* @param payload payload data to be set.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_payload(const JKHD *handle, const PayLoad *payload);

	/**
	* @brief Get current payload of the cobot.
	*
	* @param handle Control handler of the cobot. 
	* @param payload Pointer for the returned payload data.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_payload(const JKHD *handle, PayLoad *payload);

	/**
	* @brief Get current TCP position.
	*
	* @param handle Control handler of the cobot. 
	* @param tcp_position Pointer for the returned TCP position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_tcp_position(const JKHD *handle, CartesianPose *tcp_position);

	/**
	* @brief Get current joint position.
	* 
	* @param handle Control handler of the cobot. 
	* @param joint_position Pointer for the return joint position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_joint_position(const JKHD *handle, JointValue *joint_position);
	/**
	* @brief Get actual joint position.
	*
	* @param joint_position Pointer for the return joint position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_actual_joint_position(const JKHD *handle, JointValue *joint_position);

	/**
	* @brief Get the position of the end of the tool in the actual setting.
	*
	* @param tcp_position Pointer for the returned TCP position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_actual_tcp_position(const JKHD *handle, CartesianPose *tcp_position);

	/**
	* @brief Check if the cobot is now in E-Stop state.
	*
	* @param handle Control handler of the cobot. 
	* @param in_estop Pointer for the returned result.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t is_in_estop(const JKHD *handle, BOOL *in_estop);

	/**
	* @brief Check if the cobot is now on soft limit.
	*
	* @param handle Control handler of the cobot. 
	* @param on_limit Pointer for the returned result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t is_on_limit(const JKHD *handle, BOOL *on_limit);

	/**
	* @brief Check if the cobot has completed the motion (does not mean reached target).
	*
	* @param handle Control handler of the cobot. 
	* @param in_pos Pointer for the returned result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t is_in_pos(const JKHD *handle, BOOL *in_pos);

	/**
	* @brief Stop all the ongoing movements of the cobot.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t motion_abort(const JKHD *handle);

	/**
	* @brief Get motion status of the cobot.
	*
	* @param handle  Control handler of the cobot. 
	* @param status Pointer for the returned motion status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_motion_status(const JKHD* handle, MotionStatus *status);

	DLLEXPORT_API errno_t set_drag_friction_compensation_gain(const JKHD* handle, int joint, int gain);
    DLLEXPORT_API errno_t get_drag_friction_compensation_gain(const JKHD* handle, DragFrictionCompensationGainList* list);

///@}

///@name IO part
///@{
	/**
	* @brief Set multiple digital outputs (DO), specified number of DOs starting from certain index will be set.
	*
	* @param handle Control handler of the cobot.
	* @param type DO type
	* @param index Starting index of DOs to be set.
	* @param value Array of the target value.
	* @param len Number of AO to be set.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_digital_output_multi(const JKHD *handle, IOType type, int index, BOOL* value, int len);

	/**
	* @brief Set multiple analog outputs (AO), specified number of AOs starting from certain index will be set.
	*
	* @param handle Control handler of the cobot.
	* @param type Type of the analog output.
	* @param index Starting index of the analog outputs to be set.
	* @param value Array of the target value.
	* @param len Number of AO to be set.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_analog_output_multi(const JKHD *handle, IOType type, int index, float* value, int len);

	/**
	* @brief Get current status of multiple digital inputs (DI), specified number of DIs starting from certain index will be retrieved.

	* @param handle Control handler of the cobot.
	* @param type Type of the digital input.
	* @param index Starting index of the digital input.
	* @param result Pointer for the returned DI status.
	* @param len Number of DI to be queried.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_digital_input_multi(const JKHD *handle, IOType type, int index, BOOL *result, int len);

	/**
	* @brief Get current status of multiple digital inputs (DO), specified number of DOs starting from certain index will be retrieved.
	*
	* @param handle Control handler of the cobot.
	* @param type Type of the digital output.
	* @param index Starting index of the digital outputs.
	* @param result Pointer for the returned DO status.
	* @param len Number of DO to be queried.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_digital_output_multi(const JKHD *handle, IOType type, int index, BOOL *result, int len);

	/**
	* @brief Get current value of multiple analog inputs (AI), specified number of AIs starting from certain index will be retrieved.
	*
	* @param handle Control handler of the cobot.
	* @param type The type of AI.
	* @param index Starting index of the analog inputs.
	* @param result Pointer for the returned AI status.
	* @param len Number of AI to be queried.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_analog_input_multi(const JKHD *handle, IOType type, int index, float *result, int len);

	/**
	* @brief Get current value of multiple analog outputs (AO), specified number of AOs starting from certain index will be retrieved.
	*
	* @param handle Control handler of the cobot.
	* @param type The type of AO.
	* @param index Starting index of the analog outputs.
	* @param result Pointer for the returned AO status.
	* @param len Number of AO to be queried.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_analog_output_multi(const JKHD *handle, IOType type, int index, float *result, int len);

	/**
	* @brief Set the status of a digital output (DO) with specified type and index.
	*
	* @param handle Control handler of the cobot.
	* @param type Type of the digital output.
	* @param index Index of the digital output.
	* @param value Target value.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_digital_output(const JKHD *handle, IOType type, int index, BOOL value);

	/**
	* @brief Set the value of an analog output (AO) with specified type and index.
	*
	* @param handle Control handler of the cobot.
	* @param type Type of the analog output.
	* @param index Index of the analog output.
	* @param value Target value.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_analog_output(const JKHD *handle, IOType type, int index, float value);

	/**
	* @brief Get current status of a digital input (DI).
	*
	* @param handle Control handler of the cobot.
	* @param type Type of the digital input.
	* @param index Index of the digital input.
	* @param result Pointer for the returned DI status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_digital_input(const JKHD *handle, IOType type, int index, BOOL *result);

	/**
	* @brief Get current status of a digital output (DO).
	*
	* @param handle Control handler of the cobot.
	* @param type Type of the digital output.
	* @param index Index of the digital output.
	* @param result Pointer for the returned DO status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_digital_output(const JKHD *handle, IOType type, int index, BOOL *result);

	/**
	* @brief Get current value of an analog input (AI).
	*
	* @param handle Control handler of the cobot.
	* @param type Type of the analog input.
	* @param index Index of the analog input.
	* @param result Pointer for the returned AI value.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_analog_input(const JKHD *handle, IOType type, int index, float *result);

	/**
	* @brief Get current value of an analog output (AO). 
	*
	* @param handle Control handler of the cobot.
	* @param type Type of the analog output.
	* @param index Index of the analog output.
	* @param result Pointer for the returned AO value.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_analog_output(const JKHD *handle, IOType type, int index, float *result);

	/**
	* @brief Check if the extended IO modules are running.
	*
	* @param handle Control handler of the cobot.
	* @param is_running Pointer for the returned result.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t is_extio_running(const JKHD *handle, BOOL *is_running);

			/**
	 * @brief trig IO during motion, used before next motion cmd
	 * 
	 * @param rel, 0: relative to start point, 1: relate to end point
	*/
	DLLEXPORT_API errno_t set_motion_digital_output(const JKHD *handle, IOType type, int index, BOOL value, BOOL rel, double distance);

	/**
	 * @brief trig IO during motion, used before next motion cmd
	 * 
	 * @param rel, 0: relative to start point, 1: relate to end point
	*/
	DLLEXPORT_API errno_t set_motion_analog_output(const JKHD *handle, IOType type, int index, float value, BOOL rel, double distance);


///@}

///@name program part
///@{
	/**
	* @brief Start the program for the cobot. It will work only when one program has been loaded.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t program_run(const JKHD *handle);

	/**
	* @brief Pause the ongoing program for the cobot.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t program_pause(const JKHD *handle);

	/**
	* @brief Resume the program for the cobot. It will work only when one program is now in paused status.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t program_resume(const JKHD *handle);

	/**
	* @brief Abort the ongoing tasks of the cobot, the program or any movement will be terminated.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t program_abort(const JKHD *handle);

	/**
	* @brief Load a program for the cobot.
	*
	* @param handle Control handler of the cobot.
	* @param file The path of the program. For example: A/A.jks, the <file> is "A"
	* 	
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t program_load(const JKHD *handle, const char *file);

	/**
	* @brief Get current loaded program path.
	*
	* @param handle Control handler of the cobot.
	* @param file Pointer for the returned loaded program path. For example: A/A.jks, the <file> is "A"
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_loaded_program(const JKHD *handle, char *file);

	/**
	* @brief Get current execting line. It's the line number of the motion command for the program scripts. For movement 
	* commands sent via SDK, it's the motion ID defined in the movement.
	*
	* @param handle Control handler of the cobot. 
	* @param curr_line Pointer for the returned line number.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_current_line(const JKHD *handle, int *curr_line);

	/**
	* @brief Get current program status.
	*
	* @param handle Control handler of the cobot. 
	* @param status Pointer for the returned program status.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_program_state(const JKHD *handle, ProgramState *status);

	/**
	* @brief Get the user defined variables.
	*
	* @param handle Control handler of the cobot. 
	* @param vlist Pointer for the returned list of user defined variables.	

	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_user_var(const JKHD *handle, UserVariableList* vlist);

	/**
	 *@brief Set the specified user defined variable.
	*
	* @param handle Control handler of the cobot. 
	* @param v Data of the user defined variable to be set, including the ID, value and alias name.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_user_var(const JKHD *handle, UserVariable v);

	/**
	 * @brief get progrom info at once
	 */
	DLLEXPORT_API errno_t get_program_info(const JKHD *handle, ProgramInfo* info);

///@}

///@name Hand-guiding (drag) mode
///@{
	/**
	* @brief Enable/disable the hand-guiding (drag) mode.
	*
	* @param handle Control handler of the cobot. 
	* @param enable  Option to control hand-guiding mode. 0 to disable hand-guiding mode and 1 to enable.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t drag_mode_enable(const JKHD *handle, BOOL enable);

	/**
	* @brief Check if the cobot is in hand-guiding (drag) mode.
	*
	* @param handle Control handler of the cobot. 
	* @param in_drag Pointer for the returned result.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t is_in_drag_mode(const JKHD *handle, BOOL *in_drag);

///@}

///@name collision part
///@{
	/**
	* @brief Check if the cobot is in collision state.
	*
	* @param handle Control handler of the cobot. 
	* @param in_collision Pointer for the returned result.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t is_in_collision(const JKHD *handle, BOOL *in_collision);

	/**
	* @brief Recover the cobot from collision state. It will only take effect when a cobot is in collision state.
	* 
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t collision_recover(const JKHD *handle);

	/**
	* @brief Clear error status.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t clear_error(const JKHD *handle);

	/**
	* @brief Set the collision sensitivity level for the cobot.
	*
	* @param handle Control handler of the cobot. 
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
	DLLEXPORT_API errno_t set_collision_level(const JKHD *handle, const int level);

	/**
	* @brief Get current collision sensitivity level of the cobot.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_collision_level(const JKHD *handle, int *level);
///@}

///@name math part
///@{
	/**
	* @brief Calculate the inverse kinematics for a Cartesian position. It will be calclulated with the current tool, current 
	* mounting angle, and current user coordinate.
	*
	* @param handle Control handler of the cobot. 
	* @param ref_pos Reference joint position for solution selection, the result will be in the same solution space with
	* the reference joint position. 
	* @param cartesian_pose Cartesian position to do inverse kinematics calculation.
	* @param joint_pos Pointer for the returned joint position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t kine_inverse(const JKHD *handle, const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos);

	/**
	* @brief Calculate the forward kinematics for a joint position. It will be calclulated with the current tool, current 
	* mounting angle, and current user coordinate.
	*
	* @param handle Control handler of the cobot. 
	* @param joint_pos Joint position to do forward kinematics calculation.
	* @param cartesian_pose Pointer for the returned Cartesian position
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t kine_forward(const JKHD *handle, const JointValue *joint_pos, CartesianPose *cartesian_pose);


	/**
	* @brief Calculate the inverse kinematics for a Cartesian position. It will be calclulated with the current tool, current 
	* mounting angle, and current user coordinate.
	*
	* @param handle Control handler of the cobot. 
	* @param ref_pos Reference joint position for solution selection, the result will be in the same solution space with
	* the reference joint position. 
	* @param cartesian_pose Cartesian position to do inverse kinematics calculation.
	* @param joint_pos Pointer for the returned joint position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t kine_inverse_extend(const JKHD *handle, const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos, const CartesianPose *tool, const CartesianPose *userFrame);

	/**
	* @brief Calculate the forward kinematics for a joint position. It will be calclulated with the current tool, current 
	* mounting angle, and current user coordinate.
	*
	* @param handle Control handler of the cobot. 
	* @param joint_pos Joint position to do forward kinematics calculation.
	* @param cartesian_pose Pointer for the returned Cartesian position
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t kine_forward_extend(const JKHD *handle, const JointValue *joint_pos, CartesianPose *cartesian_pose, const CartesianPose *tool, const CartesianPose *userFrame);

	/**
	* @brief Convert an Euler angle in RPY to rotation matrix.
	*
	* @param handle Control handler of the cobot. 
	* @param rpy RPY value.
	* @param rot_matrix Pointer for the returned rotation matrix.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t rpy_to_rot_matrix(const JKHD *handle, const Rpy *rpy, RotMatrix *rot_matrix);

	/**
	* @brief Convert a rotation matrix to Euler angle in RPY
	*
	* @param handle Control handler of the cobot. 
	* @param rot_matrix Rotation matrix.
	* @param rpy Pointer for the returned RPY value.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t rot_matrix_to_rpy(const JKHD *handle, const RotMatrix *rot_matrix, Rpy *rpy);

	/**
	* @brief Convert a quaternion to rotation matrix.
	*
	* @param handle Control handler of the cobot. 
	* @param quaternion The quaternion data to be converted.
	* @param rot_matrix Pointer for the returned rotation matrix.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t quaternion_to_rot_matrix(const JKHD *handle, const Quaternion *quaternion, RotMatrix *rot_matrix);

	/**
	* @brief Convert a rotation matrix to quaternion.
	*
	* @param handle Control handler of the cobot. 
	* @param rot_matrix The rotation matrix to be converted.
	* @param quaternion Pointer for the returned quaternion.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t rot_matrix_to_quaternion(const JKHD *handle, const RotMatrix *rot_matrix, Quaternion *quaternion);

///@}

///@name Trajectory recording & replay
///@{
	/**
	* @brief Set trajectory recording parameters for the cobot.
	*
	* @param handle Control handler of the cobot. 
	* @param para Trajectory recording parameters.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_traj_config(const JKHD *handle, const TrajTrackPara *para);

	/**
	* @brief Get trajectory recording parameters of the cobot.
	*
	* @param handle Control handler of the cobot. 
	* @param para Pointer for the returned trajectory recording parameters.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_traj_config(const JKHD *handle, TrajTrackPara *para);

	/**
	* @brief Set the mode to enable or disable the trajectory recording (sampling) process.
	*
	* @param handle Control handler of the cobot. 
	* @param mode Trajectory recording (sampling) mode. TRUE to start trajectory recording (sampling) and FALSE to disable.
	* @param filename File name	to save the trajectory recording results.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_traj_sample_mode(const JKHD *handle, const BOOL mode, char *filename);

	/**
	* @brief Get current trajectory recording status of the cobot.
	*
	* @param handle Control handler of the cobot. 
	* @param mode Pointer for the returned status.TRUE if it's now recording, FALSE if the data recording is over or not recording. 
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_traj_sample_status(const JKHD *handle, BOOL *sample_status);

	/**
	* @brief Get all the existing trajectory recordings of the cobot.
	*
	* @param handle Control handler of the cobot. 
	* @param filename  Pointer for the returned trajectory recording files.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_exist_traj_file_name(const JKHD *handle, MultStrStorType *filename);

	/**
	* @brief Rename the specified trajectory recording file.
	*
	* @param handle Control handler of the cobot. 
	* @param src Source file name of the trajectory recording.
	* @param dest Dest file name of the trajectory recording, the length must be no more than 100 characters.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t rename_traj_file_name(const JKHD *handle, const char *src, const char *dest);

	/**
	* @brief Delete the specified trajectory file.
	*
	* @param handle Control handler of the cobot. 
	* @param filename File name of the trajectory recording.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t remove_traj_file(const JKHD *handle, const char *filename);

	/**
	* @brief  Generate program scripts from specified trajectory recording file.
	*
	* @param handle Control handler of the cobot. 
	* @param filename Trajectory recording file.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t generate_traj_exe_file(const JKHD *handle, const char *filename);
///@}

///@name servo part
///@{
	/**
	* @brief Enable or disable servo mode for the cobot.
	*
	* @param handle Control handler of the cobot. 
	* @param enable Option to enable or disable the mode. TRUE to enable servo mode，FALSE to disable servo mode.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_move_enable(const JKHD *handle, BOOL enable);

	/**
	* @brief Enable or disable servo mode for the cobot.
	*
	* @param enable Option to enable or disable the mode. TRUE to enable servo mode，FALSE to disable servo mode.	
	* @param is_block Set whether the interface is a blocking interface, TRUE for blocking interface, FALSE for non-blocking interface
	* @param robot_id ID of the target robot. Defaults to 0 if not specified.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_move_enable_extend(const JKHD *handle, BOOL enable, BOOL is_block, int robot_id);

	/**
	* @brief Check if the cobot is now in servo move mode.
	*
	* @param handle Control handler of the cobot. 
	* @param is_servo Pointer for the returned result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t is_in_servomove(const JKHD *handle, BOOL *in_servo);

	/**
	* @brief Move the cobot to the specifed joint position in servo mode. It will only work when the cobot is already in servo 
	* move mode. The servo_j command will be processed within one interpolation cycle. To ensure the cobot moves smoothly, 
	* client should send next command immediately to avoid any time delay.
	*
	* @param handle Control handler of the cobot. 
	* @param joint_pos Target joint position.
	* @param move_mode Mode for the servo_j command, to indicate it's an incremental move or absolute move.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_j(const JKHD *handle, const JointValue *joint_pos, MoveMode move_mode);

	/**
	* @brief Move the cobot to the specifed joint position in servo mode. Compared with servo_j, this command will allow
	* user to specify the step number. That is, servo_j command will be processed within specified interpolation cycles.
	* Controller will interpolate the joint poistion linearly for each specified interpolation cycle.
	*
	* @param handle Control handler of the cobot. 
	* @param joint_pos Target joint position.
	* @param move_mode Mode for the servo_j command, to indicate it's an incremental move or absolute move.
	* @param step_num  Interpolation cycles to complete the move command, step_num >= 1.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_j_extend(const JKHD *handle, const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num);

	/**
	* @brief Move the cobot to the specifed joint position in servo mode. Compared with servo_j, this command will allow
	* user to specify the step number. That is, servo_j command will be processed within specified interpolation cycles.
	* Controller will interpolate the joint poistion linearly for each specified interpolation cycle.
	*
	* @param handle Control handler of the cobot. 
	* @param joint_pos Target joint position.
	* @param move_mode Mode for the servo_j command, to indicate it's an incremental move or absolute move.
	* @param step_num  Interpolation cycles to complete the move command, step_num >= 1.
 	* @param do_info   Optional pointer to additional DO (Digital Output) information, can be nullptr if not needed.
 	* @param robot_id  ID of the target robot, default is 0.
	* @param queue_num Pointer to an integer where the function will return the current queue length 
 	*                  after adding this command. Can be nullptr if not needed.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_j_extend_queue(const JKHD *handle, const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num, int *queue_num, DOInfo *do_info, int robot_id);

	/**
	* @brief Move the cobot to the specifed Cartesian position in servo mode. Simalar with servo_j command, it will only 
	* work when the cobot is already in servo move mode and will be processed within one interpolation cycle. To ensure 
	* the cobot moves smoothly, client should send next command immediately to avoid any time delay.
	*
	* @param handle Control handler of the cobot. 
	* @param cartesian_pose Target Cartesian position.
	* @param move_mode Mode for the servo_p command, to indicate it's an incremental move or absolute move.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_p(const JKHD *handle, const CartesianPose *cartesian_pose, MoveMode move_mode);

	/**
	* @brief Move the cobot to the specifed Cartesian position in servo mode. Compared with servo_p, this command will allow
	* user to specify the step number. That is, servo_p command will be processed within specified interpolation cycles.
	* Controller will interpolate the Cartesian poistion linearly for each specified interpolation cycle.
	* 
	* @param handle Control handler of the cobot. 
	* @param cartesian_pose Target Cartesian position.
	* @param move_mode Mode for the servo_p command, to indicate it's an incremental move or absolute move.
	* @param step_num  Interpolation cycles to complete the move command, step_num >= 1.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_p_extend(const JKHD *handle, const CartesianPose *cartesian_pose, MoveMode move_mode, unsigned int step_num);

	/**
	* @brief Move the cobot to the specifed Cartesian position in servo mode. Compared with servo_p, this command will allow
	* user to specify the step number. That is, servo_p command will be processed within specified interpolation cycles.
	* Controller will interpolate the Cartesian poistion linearly for each specified interpolation cycle.
	* 
	* @param handle Control handler of the cobot. 
	* @param cartesian_pose Target Cartesian position.
	* @param move_mode Mode for the servo_p command, to indicate it's an incremental move or absolute move.
	* @param step_num  Interpolation cycles to complete the move command, step_num >= 1.
 	* @param do_info   Optional pointer to additional DO (Digital Output) information, can be nullptr if not needed.
 	* @param robot_id  ID of the target robot, default is 0.
	* @param queue_num Pointer to an integer where the function will return the current queue length 
 	*                  after adding this command. Can be nullptr if not needed.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_p_extend_queue(const JKHD *handle, const CartesianPose *cartesian_pose, MoveMode move_mode, unsigned int step_num, int *queue_num, DOInfo *do_info, int robot_id);

	/**
	* @brief Enable the EDG(external data guider) function.The EDG-related interfaces can only be used after enabling this function.
	*
	* @param en  Enable switch. true enables the EDG function, while false disables the function.
	* @param edg_stat_ip The IP address of the SDK client.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t edg_init(const JKHD *handle, BOOL en, const char* edg_stat_ip);
	/**
	* @brief Enable the EDG(external data guider) function.The EDG-related interfaces can only be used after enabling this function.
	*
	* @param en  Enable switch. true enables the EDG function, while false disables the function.
	* @param edg_stat_ip The IP address of the SDK client.
	* @param edg_port The SDK client port for receiving EDG feedback data.
	* @param edg_mode Specify the edg mode: 0:All EDG-related interfaces can be called, 1:All interfaces except edg_servo_j and edg_servo_p can be called.
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t edg_init_extend(const JKHD *handle, BOOL en, const char* edg_stat_ip, int edg_port, unsigned int edg_mode);
	/**
	* @brief High-speed acquisition of EDG feedback data from the cobot. 
	*
	* @param edg_state EDG feedback data (including joint positions, velocities, torques, Cartesian positions, CAB IO, tool IO, and sensor data).
	* @param robot_index Uses the default value, and no parameter needs to be passed.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t edg_get_stat(const JKHD *handle, EDGState *edg_state, unsigned char robot_index);
	/**
	* @brief Get the timestamp of when the EDG feedback data was issued.
	*
	* @param details Timestamp
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t edg_stat_details(const JKHD *handle, unsigned long int details[3]);
	/**
	* @brief Move the cobot to the specifed joint position in servo mode. It will only work when the cobot is already in servo 
	* move mode. 

	* @param joint_pos The target position of the robot joint motion.
	* @param move_mode Specify the motion mode: incremental or absolute.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t edg_servo_j(const JKHD *handle, const JointValue *joint_pos, MoveMode move_mode);
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
	DLLEXPORT_API errno_t edg_servo_j_extend(const JKHD *handle, const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num, unsigned char robot_index);
	/**
	* @brief Move the cobot to the specifed Cartesian position in servo mode. 
	*
	* @param cartesian_pose The target position of the robot's Cartesian motion.
	* @param move_mode Specify the motion mode: incremental or absolute.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t edg_servo_p(const JKHD *handle, const CartesianPose *cartesian_pose, MoveMode move_mode);
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
	DLLEXPORT_API errno_t edg_servo_p_extend(const JKHD *handle, const CartesianPose *cartesian_pose, MoveMode move_mode, unsigned int step_num, unsigned char robot_index);
	/**
	* @brief Disable the filter for servo move commands.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_move_use_none_filter(const JKHD *handle);

	/**
	* @brief Set 1st-order low-pass filter for servo move. It will take effect for both servo_j and servo_p commands.
	*
	* @param handle Control handler of the cobot. 
	* @param cutoffFreq Cut-off frequency for low-pass filter.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_move_use_joint_LPF(const JKHD *handle, double cutoffFreq);

	/**
	* @brief Set 3rd-order non-linear filter in joint space for servo move. It will take effect for both servo_j and 
	* servo_p commands.
	*
	* @param handle Control handler of the cobot. 
	* @param max_vr Joint speed limit, in unit °/s
	* @param max_ar Joint acceleration limit, in unit °/s^2
	* @param max_jr Joint jerk limit, in unit °/s^3	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_move_use_joint_NLF(const JKHD *handle, double max_vr, double max_ar, double max_jr);

	/**
	* @brief Set 3rd-order non-linear filter in Cartesian space for servo move. It will only take effect for servo_p
	* since the filter will be applied to Cartesian position in the commands.
	* 
	* @param handle Control handler of the cobot. 
	* @param max_vp Speed limit, in unit mm/s.s
	* @param max_ap Acceleration limit, in unit mm/s^2.
	* @param max_jp Jerk limit, in unit mm/s^3.
	* @param max_vr Orientation speed limit, in unit °/s.
	* @param max_ar Orientation acceleration limit, in unit °/s^2.
	* @param max_jr Orientation jerk limit, in unit °/s^3.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_move_use_carte_NLF(const JKHD *handle, double max_vp, double max_ap, double max_jp, double max_vr, double max_ar, double max_jr);

	/**
	* @brief Set multi-order mean filter in joint space for servo move. It will take effect for both servo_j and servo_p
	* commands.
	* 
	* @param handle Control handler of the cobot. 
	* @param max_buf Indicates the size of the mean filter buffer. If the filter buffer is set too small (less than 3), it
	* is likely to cause planning failure. The buffer value should not be too large (>100), which will bring computational
	* burden to the controller and cause planning delay; as the buffer value increases, the planning delay time increases.
	* @param kp Position filter coefficient.
	* @param kv Velocity filter coefficient.
	* @param ka Acceleration filter coefficient.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_move_use_joint_MMF(const JKHD *handle, int max_buf, double kp, double kv, double ka);

	/**
	* @brief Set velocity look-ahead filter. It’s an extended version based on the multi-order filtering algorithm with 
	* look-ahead algorithm, which can be used for joints data and Cartesian data.
	* 
	* @param handle Control handler of the cobot. 
	* @param max_buf Buffer size of the mean filter. A larger buffer results in smoother results, but with higher precision
	* loss and longer planning lag time.
	* @param kp Position filter coefficient. Reducing this coefficient will result in a smoother filtering effect, but a 
	* greater loss in position accuracy. Increasing this coefficient will result in a faster response and higher accuracy,
	* but there may be problems with unstable operation/jitter, especially when the original data has a lot of noise.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t servo_speed_foresight(const JKHD *handle, int max_buf, double kp);

///@}

///@name Torque sensor and force control
///@{

/**
 * @brief Retrieve basic information of configured force-torque sensors for the specified robot.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param infos Pointer to FTSensorBasicInfoStr structure where the retrieved sensor info will be stored.
 * @param robot_id Robot ID to query. Must be specified explicitly.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_ft_sensor_basic_info(const JKHD *handle, FTSensorBasicInfoStr *infos, int robot_id);

/**
 * @brief Retrieve soft limit rules for force-torque sensors of a robot.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param rules Pointer to FTSensorSoftLimitRuleStr structure to store retrieved rules.
 * @param robot_id Robot ID to query.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_ft_sensor_soft_limit_rules(const JKHD *handle, FTSensorSoftLimitRuleStr *rules, int robot_id);

/**
 * @brief Set soft limit rules for force-torque sensors of a robot.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param rules Pointer to FTSensorSoftLimitRuleStr defining the soft limit rules.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t set_ft_sensor_soft_limit_rules(const JKHD *handle, const FTSensorSoftLimitRuleStr *rules, int robot_id);

/**
 * @brief Set filter parameters for a specified force-torque sensor.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param sensor_id ID of the force-torque sensor to configure.
 * @param filter Pointer to an array of double containing filter parameters.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t set_ft_sensor_filter(const JKHD *handle, int sensor_id, const double *filter);

/**
 * @brief Retrieve filter parameters of a specified force-torque sensor.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param sensor_id ID of the sensor.
 * @param filter Pointer to a double array to store retrieved filter parameters.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_ft_sensor_filter(const JKHD *handle, int sensor_id, double *filter);

/**
 * @brief Get the current on/off mode of a force-torque sensor.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param sensor_id ID of the sensor.
 * @param mode Pointer to int to receive mode: 1 for on, 0 for off.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_ft_sensor_mode(const JKHD *handle, int sensor_id, int *mode);

/**
 * @brief Zero the specified force-torque sensor.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param sensor_id ID of the sensor to zero.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t set_ft_sensor_zero(const JKHD *handle, int sensor_id);

/**
 * @brief Set contact force threshold for a specified sensor.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param sensor_id ID of the sensor.
 * @param threshold Pointer to FTSensorThresholdStr structure containing threshold values.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t set_contact_force_threshold(const JKHD *handle, int sensor_id, const FTSensorThresholdStr *threshold);

/**
 * @brief Retrieve contact force threshold for a specified sensor.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param sensor_id ID of the sensor.
 * @param threshold Pointer to FTSensorThresholdStr to store retrieved threshold.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_contact_force_threshold(const JKHD *handle, int sensor_id, FTSensorThresholdStr *threshold);

/**
 * @brief Set the reference point of the force-torque sensor.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param ref_point Pointer to CartesianTran specifying [x,y,z] offset in meters.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t set_ft_sensor_ref_point(const JKHD *handle, const CartesianTran *ref_point, int robot_id);

/**
 * @brief Get the reference point of the force-torque sensor.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param ref_point Pointer to CartesianTran to receive offset.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_ft_sensor_ref_point(const JKHD *handle, CartesianTran *ref_point, int robot_id);

/**
 * @brief Retrieve current force/torque data from a sensor.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param sensor_id ID of the sensor.
 * @param type Data type to retrieve:
 *             1 = actTorque (compatibility),
 *             2 = original readings,
 *             3 = real data without gravity and bias.
 * @param data Pointer to FTSensorDataStr to store the result.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_ft_sensor_data(const JKHD *handle, int sensor_id, int type, FTSensorDataStr *data);

/**
 * @brief Get payload information associated with a sensor.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param sensor_id ID of the sensor.
 * @param payload Pointer to PayLoad structure to receive payload info.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_ft_sensor_payload(const JKHD *handle, int sensor_id, PayLoad *payload);

/**
 * @brief Set the payload ID for a specific force-torque sensor.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param sensor_id ID of the sensor.
 * @param payload_id Payload configuration ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t set_ft_sensor_payload(const JKHD *handle, int sensor_id, int payload_id);

/**
 * @brief Get IDs of force-torque sensors linked to the robot.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param sensor_ids Pointer to FTLinkedSensorIDStr to receive sensor IDs.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_linked_ft_sensor_id(const JKHD *handle, FTLinkedSensorIDStr *sensor_ids, int robot_id);

/**
 * @brief Add and configure a new force-torque sensor.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param config Pointer to FTSensorConfigStr with sensor configuration.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t add_ft_sensor(const JKHD *handle, const FTSensorConfigStr *config, int robot_id);

/**
 * @brief Remove a force-torque sensor from the system.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param sensor_id ID of the sensor to remove.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t remove_ft_sensor(const JKHD *handle, int sensor_id, int robot_id);

/**
 * @brief Set velocity limits for force control mode.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param limit Pointer to VelocityLimit structure with linear and angular limits.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t set_force_ctrl_vel_limit(const JKHD *handle, const VelocityLimit *limit, int robot_id);

/**
 * @brief Get velocity limits for force control mode.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param limit Pointer to VelocityLimit to receive velocity limits.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_force_ctrl_vel_limit(const JKHD *handle, VelocityLimit *limit, int robot_id);

/**
 * @brief Set velocity limits for approaching mode.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param limit Pointer to VelocityLimit specifying limits.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t set_approaching_vel_limit(const JKHD *handle, const VelocityLimit *limit, int robot_id);

/**
 * @brief Get velocity limits for approaching mode.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param limit Pointer to VelocityLimit to receive limits.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_approaching_vel_limit(const JKHD *handle, VelocityLimit *limit, int robot_id);

/**
 * @brief Set custom force control configuration.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param config Pointer to AdmitCtrlType structure defining control behavior.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t set_cst_force_ctrl_config(const JKHD *handle, const AdmitCtrlType *config, int robot_id);

/**
 * @brief Retrieve custom force control configuration.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param config Pointer to AdmitCtrlType to receive configuration.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_cst_force_ctrl_config(const JKHD *handle, AdmitCtrlType *config, int robot_id);

/**
 * @brief Set tolerance for custom force control.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param tol Pointer to FTxyz structure specifying tolerances.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t set_cst_force_ctrl_tol(const JKHD *handle, const FTxyz *tol, int robot_id);

/**
 * @brief Get tolerance for custom force control.
 *
 * @param handle Pointer to control handler of the cobot.
 * @param tol Pointer to FTxyz to receive tolerance values.
 * @param robot_id Robot ID.
 *
 * @return errno_t Returns ERR_SUCC if successful, other values indicate failure.
 */
DLLEXPORT_API errno_t get_cst_force_ctrl_tol(const JKHD *handle, FTxyz *tol, int robot_id);




	/**
	* @brief Set the type of torque sensor.
	* 
	* @param handle Control handler of the cobot. 
	* @param sensor_brand Type of the torque sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_torsenosr_brand(const JKHD *handle, int sensor_brand);

	/**
	* @brief Get the type of torque sensor.
	* 
	* @param handle Control handler of the cobot. 
	* @param sensor_brand  Pointer to the returned torque sensor type.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_torsenosr_brand(const JKHD *handle, int *sensor_brand);

	/**
	* @brief Set mode to turn on or turn off the torque sensor.
	* 
	* @param handle Control handler of the cobot.
	* @param sensor_mode Mode of the torque sensor, 1 to turn on and 0 to turn off the torque sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_torque_sensor_mode(const JKHD *handle, int sensor_mode);

	/**
	*@brief check if torque sensor enabled or not
	*/
	DLLEXPORT_API errno_t get_torque_sensor_mode(const JKHD *handle, int* sensor_mode);

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Set parameters for admittance control of the cobot.
	* 
	* @param handle Control handler of the cobot. 
	* @param axis ID of the axis to be controlled, axis with ID 0 to 5 corresponds to x, y, z, Rx, Ry, Rz.
	* @param opt  Enable flag. 0: disable, 1: enable.
	* @param ftUser  Force to move the cobot in maximum speed.
	* @param ftReboundFK  Ability to go back to initial position.
	* @param ftConstant Set to 0 when operate manually.
	* @param ftNnormalTrack Set to 0 when operate manually.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_admit_ctrl_config(const JKHD *handle, const int axis, const int opt, const double ftUser, const double ftConstant, const int ftNnormalTrack, const double ftReboundFK);

	/**
	* @brief Start to identify payload of the torque sensor.
	* 
	* @param handle Control handler of the cobot. 
	* @param joint_pos End joint position of the trajectory.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t start_torq_sensor_payload_identify(const JKHD *handle, const JointValue *joint_pos);

	/**
	* @brief Get the status of torque sensor payload identification
	* 
	* @param handle Control handler of the cobot. 
	* @param identify_status Pointer of the returned result. 0: done，1: no identification result ready to read，2: error.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_torq_sensor_identify_staus(const JKHD *handle, int *identify_status);

	/**
	* @brief Get identified payload of the torque sensor.
	*
	* @param handle Control handler of the cobot. 
	* @param payload Pointer to the returned identified payload in kg, mm.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_torq_sensor_payload_identify_result(const JKHD *handle, PayLoad *payload);

	/**
	* @brief Set the payload for the torque sensor in kg, mm.
	*
	* @param handle Control handler of the cobot. 
	* @param payload Payload of torque sensor.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_torq_sensor_tool_payload(const JKHD *handle, const PayLoad *payload);

	/**
	* @brief Get current payload of the torque sensor in kg, mm.
	*
	* @param handle Control handler of the cobot. 
	* @param payload Pointer to the returned payload.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_torq_sensor_tool_payload(const JKHD *handle, PayLoad *payload);

	/**
	* @brief Enable or disable tool drive. It will only work when a torque sensor is equiped. THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* 
	* @param handle Control handler of the cobot. 
	* @param enable_flag Option to indicate enable or disable the admittance control. 1 to enable and 0 to disable
	* the admittance control.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t enable_admittance_ctrl(const JKHD *handle, const int enable_flag);

	/**
	* @brief Enable or disable the tool drive. customer may drag robot by the end flange or the tools mounted on it
	* 
	* @param handle Control handler of the cobot. 
	* @param enable_flag Option to indicate enable or disable: 1 to enable and 0 to disable
	* the admittance control.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t enable_tool_drive(const JKHD *handle, const int enable_flag);

	/**
	* @brief Set tool drive configuration.
	* 
	* @param handle Control handler of the cobot. 
	* @param cfg Configuration for the tool drive.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_tool_drive_config(const JKHD *handle, ToolDriveConfig cfg);

	/**
	* @brief Get current configuration for tool drive.
	* 
	* @param handle Control handler of the cobot. 
	* @param cfg Pointer for the returned tool drive configuration.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_tool_drive_config(const JKHD *handle, RobotToolDriveCtrl* cfg);

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Set compliance control type and sensor initialization status.
	*
	* @param handle Control handler of the cobot. 
	* @param sensor_compensation Option to enable or disable the sensor compensation, 1: on, 0: off.
	* @param compliance_type Compliance control type: 0:disabled, 1:constant force control, 2:speed compliance control.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_compliant_type(const JKHD *handle, int sensor_compensation, int compliance_type);

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Get compliance control type and sensor initialization status.
	* 
	* @param handle Control handler of the cobot. 
	* @param sensor_compensation Pointer for the returned sensor compensation option.
	* @param compliance_type Pointer for the returned compliance control type.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_compliant_type(const JKHD *handle, int *sensor_compensation, int *compliance_type);

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Get admitrance control configurations.
	*
	* @param handle Control handler of the cobot. 
	* @param admit_ctrl_cfg Pointer for the returned admittance control configurations.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_admit_ctrl_config(const JKHD *handle, RobotAdmitCtrl *admit_ctrl_cfg);

	/**
	* @brief Setup the communication for the torque sensor.
	*
	* @param handle Control handler of the cobot. 
	* @param type Commnunication type of the torque sensor, 0: type I/II/III/IV/V sensor, 1: type VI sensor.
	* @param ip_addr IP address of the torque sensor. Only for type I/III/IV sensor
	* @param port Port for the torque sensor. Only for type I/III/IV sensor
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_torque_sensor_comm(const JKHD *handle, const int type, const char *ip_addr, const int port);

	/**
	* @brief Get the communication settings of the torque sensor.
	*
	* @param handle Control handler of the cobot. 
	* @param type Pointer for the returned commnunication type.
	* @param ip_addr Pointer for the returned IP address.
	* @param port Pointer for the returned port.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_torque_sensor_comm(const JKHD *handle, int *type, char *ip_addr, int *port);

	/**
	* @brief Set the torque sensor low-pass filter parameter for force control.
	*
	* @param handle Control handler of the cobot. 
	* @param torque_sensor_filter Filter parameter, unit：Hz
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_torque_sensor_filter(const JKHD *handle, const float torque_sensor_filter);

	/**
	* @brief Get the filter parameter of force control.
	*
	* @param handle Control handler of the cobot. 
	* @param torque_sensor_filter Pointer for the returned filter parameter, unit：Hz
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_torque_sensor_filter(const JKHD *handle, float *torque_sensor_filter);

	/**
	* @brief Set soft force or torque limit for the torque sensor.
	*
	* @param handle Control handler of the cobot. 
	* @param torque_sensor_soft_limit Soft limit, fx/fy/fz is force limit in unit N and tx/ty/tz is torque limit unit：N*m.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_torque_sensor_soft_limit(const JKHD *handle, const FTxyz torque_sensor_soft_limit);

	/**
	* @brief Get current soft force or torque limit of the torque sensor.
	*
	* @param handle Control handler of the cobot. 
	* @param torque_sensor_soft_limit Pointer for the returned soft limits.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_torque_sensor_soft_limit(const JKHD *handle, FTxyz *torque_sensor_soft_limit);

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Disabled force control of the cobot.
	*
	* @param handle Control handler of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t disable_force_control(const JKHD *handle);

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Set the parameters for velocity complianance control.
	*
	* @param handle Control handler of the cobot. 
	* @param vel Prameters for velocity compliance control.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_vel_compliant_ctrl(const JKHD *handle, const VelCom *vel);

	/**
	* @brief THIS IS A COMPATIBILITY INTERFACE, NOT RECOMMANDED TO USE.
	* Set condition for compliance control.
	*
	* @param handle Control handler of the cobot. 
	* @param ft the max force, if over limit, the robot will stop movement
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_compliance_condition(const JKHD *handle, const FTxyz *ft);

	/**
	* @brief Set the approaching speed limit of constant force compliant control.
	*
	* @param handle Control handler of the cobot. 
	* @param vel Line speed, mm/s.
	* @param angularVel Angular speed rad/s.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_approach_speed_limit(const JKHD *handle, double vel, double angularVel);
	/**
	* @brief Get the approaching speed limit of constant force compliant control.
	*
	* @param handle Control handler of the cobot. 
	* @param vel Pointer for the returned line speed, mm/s.
	* @param angularVel Pointer for the returned angular speed rad/s.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_approach_speed_limit(const JKHD *handle, double* vel, double* angularVel);
	/**
	* @brief Set the force/torque tolerance of constant force compliant control.
	*
	* @param handle Control handler of the cobot. 
	* @param force Force tolerance, N.
	* @param torque Torque tolerance, Nm.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_ft_tolerance(const JKHD *handle, double force, double torque);
	/**
	* @brief Get the force/torque tolerance of constant force compliant control.
	*
	* @param handle Control handler of the cobot. 
	* @param force Pointer for the returned force tolerance, N.
	* @param torque Pointer for the returned torque tolerance, Nm.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_ft_tolerance(const JKHD *handle, double* force, double* torque);

	/**
	@brief enable or disable force control
	*/
	DLLEXPORT_API errno_t set_ft_ctrl_mode(const JKHD *handle, BOOL mode);
	/**
	@brief get force control state
	*/
	DLLEXPORT_API errno_t get_ft_ctrl_mode(const JKHD *handle, BOOL* mode);
	/**
	@brief set force control config
	*/
	DLLEXPORT_API errno_t set_ft_ctrl_config(const JKHD *handle, AdmitCtrlType cfg);
	/**
	@brief get force control config
	*/
	DLLEXPORT_API errno_t get_ft_ctrl_config(const JKHD *handle, RobotAdmitCtrl* cfg);

	/**
	* @brief Set the coordinate or frame for the force control.
	*
	* @param handle Control handler of the cobot. 
	* @param ftFrame Coordinate or frame option. 0: tool frame, 1:world frame.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_ft_ctrl_frame(const JKHD *handle, FTFrameType ftFrame);

	/**
	* @brief Get the coordinate or frame of the force control.
	*
	* @param handle Control handler of the cobot. 
	* @param ftFrame Pointer for the returned frame option.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_ft_ctrl_frame(const JKHD *handle, FTFrameType *ftFrame);

	/**
	* @brief Get the torque sensor data of specified type.
	*
	* @param handle Control handler of the cobot. 
	* @param type Type of the data to be retrieved: 1 for actTorque (compatibility interface),  2 for original readings,  3 for real data without gravity and bias.
	* @param data Pointer for the returned feedback data
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_torque_sensor_data(const JKHD *handle, int type, TorqSensorData* data);

	/**
	 * @brief Trigger sensor zeroing and blocking for 0.5 seconds
	 *
	 * @param handle Control handler of the cobot. 
	 *
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	DLLEXPORT_API errno_t zero_end_sensor(const JKHD *handle);

	/**
	* @brief Get the tool drive mode and state.
	*
	* @param handle Control handler of the cobot. 
	* @param enable Pointer for the returned value that indicating if the tool drive mode is enabled or not.
	* @param state Pointer for the returned value that indicating if current state of tool drive triggers singularity point, speed, joint limit warning.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_tool_drive_state(const JKHD *handle, int* enable, int *state);

	/**
	* @brief Get coordinate system for tool drive.
	*
	* @param handle Control handler of the cobot. 
	* @param ftFrame Pointer for the return coordinate system, 0 for tcp and 1 for userframe.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_tool_drive_frame(const JKHD *handle, FTFrameType *ftFrame);

	/**
	* @brief Set the the coordinate system for tool drive.
	*
	* @param handle Control handler of the cobot. 
	* @param ftFrame Coordinate system option for tool drive, 0 for tcp and 1 for userframe.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_tool_drive_frame(const JKHD *handle, FTFrameType ftFrame);

	/**
	* @brief Get the sensitivity setting for fusion drive.
	*
	* @param handle Control handler of the cobot. 
	* @param level Pointer for the returned sensitivity level,which ranges within [0,5] and 0 means off, the smaller the less sensitive.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_fusion_drive_sensitivity_level(const JKHD *handle, int *level);

	/**
	* @brief Set the sensitivity level for the fusion drive.
	*
	* @param handle Control handler of the cobot. 
	* @param level Sensitivity level, which ranges within [0,5] and 0 means off, the smaller the less sensitive.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_fusion_drive_sensitivity_level(const JKHD *handle, int level);

	/**
	* @brief Get the warning range of motion limit (singularity point and joint limit)
	*
	* @param handle Control handler of the cobot. 
	* @param warning_range Range level, 1-5, the smaller the larger the warning range is.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_motion_limit_warning_range(const JKHD *handle, int *warning_range);

	/**
	* @brief Set the warning range for motion limit, like singularity point and joint limit.
	*
	* @param handle Control handler of the cobot. 
	* @param warning_range Range level, 1-5, the smaller the larger the warning range is.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_motion_limit_warning_range(const JKHD *handle, int warning_range);

	/**
	* @brief Get force control speed limit.
	*
	* @param handle Control handler of the cobot. 
	* @param speed_limit Line speed limit, mm/s
	* @param angular_speed_limit Angular speed limit, rad/s	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_compliant_speed_limit(const JKHD *handle, double* speed_limit, double* angular_speed_limitangularVel);

	/**
	* @brief Set force control speed limit.
	*
	* @param handle Control handler of the cobot. 
	* @param speed_limit Line speed limit, mm/s.
	* @param angular_speed_limit Angular speed limit, rad/s.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_compliant_speed_limit(const JKHD *handle, double speed_limit, double angular_speed_limit);

	/**
	* @brief Get the torque reference center.
	*
	* @param handle Control handler of the cobot. 
	* @param ref_point 0 represents the sensor center, 1 represents TCP	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_torque_ref_point(const JKHD *handle, int *ref_point);

	/**
	* @brief Set the torque reference center.
	*
	* @param handle Control handler of the cobot. 
	* @param ref_point 0 represents the sensor center, 1 represents TCP	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_torque_ref_point(const JKHD *handle, int ref_point);

	/**
	* @brief Get sensor sensitivity.
	*
	* @param handle Control handler of the cobot. 
	* @param threshold Torque or force threshold for each axis, ranging within [0, 1]. The larger the value, the less sensitive the sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_end_sensor_sensitivity_threshold(const JKHD *handle, FTxyz *threshold);

	/**
	* @brief Set the sensor sensitivity.
	*
	* @param handle Control handler of the cobot. 
	* @param threshold Torque or force threshold for each axis, ranging within [0, 1]. The larger the value, the less sensitive the sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_end_sensor_sensitivity_threshold(const JKHD *handle, FTxyz threshold);

	/**
	 * @brief set force stop condition, only execute once for next motion command
	 */
	DLLEXPORT_API errno_t set_force_stop_condition(const JKHD *handle, ForceStopConditionList condition);

///@}

///@name SDK support
///@{
	/**
	* @brief Get the version number of SDK.
	*
	* @param handle Control handler of the cobot. 
	* @param version Pointer for the returned SDK verion.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_sdk_version(const JKHD *handle, char *version);

	/**
	* @brief Get the IP address of the control cabinet.
	*
	* @param controller_name  Controller name. 
	* @param ip_list Controller ip list, controller name for the specific value to return the name of the corresponding controller IP address, controller name is empty, return to the segment class of all the controller IP address
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_controller_ip(const JKHD *handle, char *controller_name, char *ip_list);

	/**
	* @brief Set the path to the error code file, if you need to use the get_last_error interface you need to set the path to the error code file, if you don't use the get_last_error interface, you don't need to set the interface.
	* 
	* @param handle Control handler of the cobot.
	* @param path  File path for the error code.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_errorcode_file_path(const JKHD *handle, char *path);

	/**
	* @brief Get last error code from the system.
	*
	* @param handle Control handler of the cobot.
	* @param code Pointer for the returned error code.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_last_error(const JKHD *handle, ErrorCode *code);

	/**
	* @brief Enable or disable the debug mode for SDK control. If enabled, SDK log may contains more detailed information. 
	* @deprecated Only useful before SDK 2.1.12
	* 
	* @param handle Control handler of the cobot.
	* @param mode Option to enable or disable the debug mode. 1: enable, 0: disable.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_debug_mode(const JKHD *handle, BOOL mode);

	/**
	* @brief Set the reaction behavior of the cobot when connection to SDK is lost.
	* 
	* @param handle Control handler of the cobot. 
	* @param millisecond Timeout of connection loss, unit: ms.
	* @param mnt Reaction behavior type.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_network_exception_handle(const JKHD *handle, float millisecond, ProcessType mnt);

	/**
	* @brief Set file path for the SDK log.
	*
	* @param handle Control handler of the cobot. 
	* @param filepath File path of the log.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_SDK_filepath(const JKHD *handle, char *filepath);

	/**
	* @brief Get the SDK log path.
	* 
	* @param handle Control handler of the cobot. 
	* @param filepath Pointer for the returned path.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_SDK_filepath(const JKHD *handle, char *filepath, int size);
///@}

///@name TIO part
///@{
	/**
	* @brief Set voltage parameter for TIO of the cobot. It only takes effect for TIO with hardware version 3.
	* 
	* @param handle Control handler of the cobot. 
	* @param vout_enable Option to enable voltage output. 0:turn off， 1:turn on.
	* @param vout_vol Option to set output voltage. 0: 24V, 1:12V.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_tio_vout_param(const JKHD* handle, int vout_enable, int vout_vol);

	/**
	* @brief Get voltage parameter of TIO of the cobot. It only takes effect for TIO with hardware version 3.
	* 
	* @param handle Control handler of the cobot. 
	* @param vout_enable Pointer for the returned voltage enabling option.
	* @param vout_vol Pointer for the returned output voltage option.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_tio_vout_param(const JKHD* handle, int* vout_enable, int* vout_vol);

	/**
	* @brief Add or modify the signal for TIO RS485 channels.
	*
	* @param handle Control handler of the cobot. 
	* @param sign_info Definition data of the signal.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t add_tio_rs_signal(const JKHD* handle, SignInfo sign_info);

	/**
	* @brief Delete the specified signal for TIO RS485 channel.
	* 
	* @param handle Control handler of the cobot. 
	* @param sig_name Signal name.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t del_tio_rs_signal(const JKHD* handle, const char* sig_name);

	/**
	* @brief Send a command to the specified RS485 channel.
	*
	* @param handle Control handler of the cobot. 
	* @param chn_id ID of the RS485 channel in TIO.
	* @param data Command data.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t send_tio_rs_command(const JKHD* handle, int chn_id, uint8_t* cmdbuf, int bufsize);

	/**
	* @brief Get all the defined signals in TIO module.
	* 
	* @param handle Control handler of the cobot. 
	* @param sign_info_array Pointer for the returned signal list.
	* @param array_len Pointer for the size of the returned signal list.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_rs485_signal_info(const JKHD* handle, SignInfo* sign_info_array, int *array_len);

	/**
	* @brief Set tio mode.
	*
	* @param handle Control handler of the cobot. 
	* @param pin_type Type of the TIO pin. 0 for DI Pins, 1 for DO Pins, 2 for AI Pins.
	* @param pin_mode Mode of the TIO pin.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_tio_pin_mode(const JKHD* handle, int pin_type, int pin_mode);

	/**
	* @brief Get mode of the TIO pin of specified type.
	* 
	* @param handle Control handler of the cobot. 
	* @param pin_type Type of the TIO pin. 0 for DI Pins, 1 for DO Pins, 2 for AI Pins.
	* @param pin_mode Pointer for the returned TIO mode.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_tio_pin_mode(const JKHD* handle, int pin_type, int* pin_mode);

	/**
	* @brief Setup communication for specified RS485 channel.
	* 
	* @param handle Control handler of the cobot. 
	* @param mod_rtu_com Modbus RTU communication settings.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_rs485_chn_comm(const JKHD* handle, ModRtuComm mod_rtu_com);

	/**
	* @brief Get RS485 commnunication setting.
	* 
	* @param handle Control handler of the cobot. 
	* @param mod_rtu_com Pointer for the returned communication settings.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_rs485_chn_comm(const JKHD* handle, int chn_id, ModRtuComm* mod_rtu_com);

	/**
	* @brief Set the mode for specified RS485 channel.
	*
	* @param handle Control handler of the cobot. 
	* @param chn_id Channel id. 0 for RS485H, channel 1; 1 for RS485L, channel 2.
	* @param chn_mode Mode to indicate the usage of RS485 channel. 0 for Modbus RTU, 1 for Raw RS485, 2 for torque sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t set_rs485_chn_mode(const JKHD* handle, int chn_id, int chn_mode);

	/**
	* @brief Get the mode of specified RS485 channel.
	* 
	* @param handle Control handler of the cobot. 
	* @param chn_id Channel id. 0: RS485H, channel 1; 1: RS485L, channel 2
	* @param chn_mode Pointer for the returned mode. 0: Modbus RTU, 1: Raw RS485, 2, torque sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_rs485_chn_mode(const JKHD* handle, int chn_id, int* chn_mode);
	/**
	* @brief Perform hardware-level calibration of the TIO sensor.
	* 
	* @param type 0: No effect, 1: Perform calibration, 2: Query the current calibration value.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t tio_sensor_calib(const JKHD* handle, int type);
	/**
	* @brief Update the signal for TIO RS485 channels.
	*
	* @param handle Control handler of the cobot. 
	* @param sign_info Definition data of the signal.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t update_tio_rs_signal(const JKHD* handle, SignInfo_simple sign_info);
///@}

///@name FTP part
///@{
	/**
	* @brief Initialize the FTP client in SDK.
	* 
	* @param handle Control handler of the cobot. 
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t init_ftp_client(const JKHD *handle);

	/**
	* @brief Initialize the FTP with SSL (need APP connected and controller support).
	* 
	* @param handle Control handler of the cobot. 
	* @param password Password for authentification.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t init_ftp_client_with_ssl(const JKHD* handle, char* password);

	/**
	* @brief Close the FTP connection with control cabinet.	
	*
	* @param handle Control handler of the cobot. 
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t close_ftp_client(const JKHD *handle);

	/**
	* @brief Transfer a file or directory from local machine to cobot controller.
	*
	* @param handle Control handler of the cobot. 
	* @param remote Remote absolute path in cobot controller.
	* @param local Path of the local file to be transfered.
	* @param opt Option to indicate the type of the file. 1: single file, 2: folder.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t download_file(const JKHD *handle, char *local, char *remote, int opt);

	/**
	* @brief Upload file from controller to the local machine.
	* 
	* @param handle Control handler of the cobot. 
	* @param remote Remote absolute path in cobot controller.
	* @param local Path in the local machine to save the file.
	* @param opt Option to indicate the type of the file. 1: single file, 2: folder.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t upload_file(const JKHD *handle, char *local, char *remote, int opt);

	/**
	* @brief Delete a file in control cabinet via FTP.
	*
	* @param handle Control handler of the cobot. 
	* @param remote File name in control cabinet.
	* @param opt Option to indicate the type of the file. 1: single file, 2: folder.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t del_ftp_file(const JKHD *handle, char *remote, int opt);

	/**
	* @brief Rename the file in cobot controller via FTP server.
	*
	* @param handle Control handler of the cobot. 
	* @param remote Original file in control cabinet to be renamed.
	* @param des Target file name.
	* @param opt Option to indicate the type of the file. 1: single file, 2: folder.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t rename_ftp_file(const JKHD *handle, char *remote, char *des, int opt);

	/**
	* @brief Get the directory of the FTP service. 
	* 
	* @param handle Control handler of the cobot. 
	* @param remotedir Pointer for the returned FIP directory. like "/track/" or "/program/"
	* @param type Type of the file. 0: file and folder, 1: single file, 2: folder
	* @param ret Returned structure of the directory in string format.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	DLLEXPORT_API errno_t get_ftp_dir(const JKHD *handle, const char *remotedir, int type, char *ret);

///@}


#ifdef __cpluscplus
}
#endif

#undef DLLEXPORT_API
#endif
