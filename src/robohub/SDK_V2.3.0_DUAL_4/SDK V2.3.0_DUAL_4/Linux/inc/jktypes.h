/**
 * @last update Nov 30 2021
 * @Maintenance star@jaka
 */
#ifndef _JHTYPES_H_
#define _JHTYPES_H_

#define TRUE 1
#define FALSE 0

#include <stdio.h>
#include <stdint.h>


#define JAKA_ROBOT_MAX_JOINT 7
#define LEFT 0
#define RIGHT 1
#define DUAL -1


typedef int BOOL;    // SDK bool type
typedef int JKHD;    // SDK handler for C
typedef int errno_t; // SDK error code feedback

#define MAX_PAYLOAD_CNT 16
#define MAX_FTSENSOR_CNT 20

/**
 * @brief cartesian position without orientation
 */
typedef struct {
    double x; ///< x axis，unit: mm
    double y; ///< y axis，unit: mm
    double z; ///< z axis，unit: mm
} CartesianTran;

/**
 * @brief cartesian orientation
 */
typedef struct {
    double rx; ///< x axis，unit：rad
    double ry; ///< y axis，unit：rad
    double rz; ///< z axis，unit：rad
} Rpy;

/**
 * @brief quaternion for orientation
 */
typedef struct {
    double s;
    double x;
    double y;
    double z;
} Quaternion;

/**
 *@brief cartesian position with orientation
 */
typedef struct {
    CartesianTran tran; ///< cartesian translation
    Rpy rpy;            ///< cartesian rotation

} CartesianPose;

typedef struct {
    CartesianPose pose;
    char name[100];
    int id;
} ToolInfo;

typedef struct {
    CartesianPose pose;
    char name[100];
    int id;
} UserFrameInfo;

/**
 * @brief rotation marix
 */
typedef struct {
    CartesianTran x; ///< x component
    CartesianTran y; ///< y component
    CartesianTran z; ///< z component
} RotMatrix;

/**
 * @brief program executing state enum
 */
typedef enum {
    PROGRAM_IDLE,    ///< idle
    PROGRAM_RUNNING, ///< running
    PROGRAM_PAUSED   ///< paused, able to resume
} ProgramState;

/**
 * @brief program info
 */
typedef struct {
    int logic_line;             ///< program script executing line
    int motion_line;            ///< executing motion CMD id
    char file[100];             ///< current program file
    ProgramState program_state; ///< program executing state
} ProgramInfo;

/**
 *@brief Controller User Variable Struct
 *@param id controller inner usage
 *@param value value type always double
 *@param alias variable alias which is less than 100 bytes
 */
typedef struct {
    int id;
    double value;
    char alias[100];
} UserVariable;

/**
 * @brief number of UserVariable is fixed to 100
 */
typedef struct {
    UserVariable v[100];
} UserVariableList;

/**
 * @brief operations when lost communication
 */
typedef enum {
    MOT_KEEP,  ///< no change
    MOT_PAUSE, ///< pause
    MOT_ABORT  ///< abort
} ProcessType;

//
typedef struct {
    int major;
    int minor;
    int patch;
    int suffix;
    char version[50];
    char scb[50];
    char servo[50];
} ControllerVersion;

typedef struct{
    int major;
    int minor;
    int patch;
    int suffix;
    char msg[50];
} SDKVersion;

/**
 * @brief coordinate type
 */
typedef enum {
    COORD_BASE,  ///< robot base coordinate
    COORD_JOINT, ///< robot joint coordinate
    COORD_TOOL   ///< robot tool coordinate
} CoordType;

/**
 * @brief move mode
 */
typedef enum {
    ABS = 0,
	INCR,
	CONTINUE,
    STOP
} MoveMode;

typedef enum { 
    GlobalPlanner_disable = -1,
    GlobalPlanner_T = 0, 
    GlobalPlanner_S = 1 
} MotionPlannerType;

typedef enum { TIO_VOUT_24V = 0, TIO_VOUT_12V = 1 } TIO_VOUT;

typedef struct {
    BOOL enable;
    TIO_VOUT v;
} TIOInfo;

typedef struct {
    int pin_type;
    int pin_mode;
} TIO_pin;

/**
 * @brief payload
 */
typedef struct {
    
    double mass;            ///< mass, unit：kg
    CartesianTran centroid; ///< centroid, unit：mm
    int payload_id;         ///< payload identifier
} PayLoad;

/**
 * @brief joint position
 */
typedef struct {
    double jVal[JAKA_ROBOT_MAX_JOINT]; ///< each joint，unit：rad
} JointValue;

typedef enum{
    JogStop = 0,
    JogCONT = 1,
    JogINCR = 2,
    JogABS = 3,
} JogMode;

typedef struct {
    int aj_num;
    JogMode jog_mode;
    CoordType coord_type;
    double vel_cmd;
    double pos_cmd;
} JogParam;

/**
 *@brief joint move param
 */
typedef struct {
    int id;            ///< motion cmd id, range limit: [0, 5000], set to -1 if you want controller to set automatically
    BOOL is_block;     ///< block until this cmd is done
    JointValue joints; ///< targe joint value
    MoveMode mode;     ///< motion mode
    double vel;        ///< velocity
    double acc;        ///< acceleration, set to 90 if you have no idea
    double tol;        ///< tolerance, used for blending. set to 0 if you want to reach a fine point
} MoveJParam;

typedef struct {
    int id;                ///< motion cmd id, range limit: [0, 5000], set to -1 if you want controller to set automatically
    BOOL is_block;         ///< block until this cmd is done
    CartesianPose end_pos; ///< taget position
    MoveMode move_mode;    ///< motion mode
    double vel;            ///< velocity
    double acc;            ///< acceleration, set to 500 if you have no idea
    double tol;            ///< tolerance, used for blending. set to 0 if you want to reach a fine point
    double ori_vel;        ///< set to 3.14 if you have no idea
    double ori_acc;        ///< set to 12.56 if you have no idea
} MoveLParam;

typedef struct {
    int id;                ///< motion cmd id, range limit: [0, 5000], set to -1 if you want controller to set automatically
    BOOL is_block;         ///< block until this cmd is done
    CartesianPose mid_pos; ///< mid position
    CartesianPose end_pos; ///< end position
    MoveMode move_mode;    ///< motion mode
    double vel;            ///< velocity
    double acc;            ///< acceleration, set to 500 if you have no idea
    double tol;            ///< tolerance, used for blending. set to 0 if you want to reach a fine point

    double circle_cnt; ///< circule count
    int circle_mode;   ///< clock wise or counter clock wise
} MoveCParam;

/**
 * @brief IO type
 */
typedef enum {
    IO_CABINET,        ///< cabinet IO
    IO_TOOL,           ///< tool IO
    IO_EXTEND,         ///< extended IO
    IO_REALY,          ///< relay IO，only cab v3 supports relay DO
    IO_MODBUS_SLAVE,   ///< Modbus slave station IO, index start from 0.
    IO_PROFINET_SLAVE, ///< Profinet slave station IO, index start from 0.
    IO_EIP_SLAVE       ///< ETHRENET/IP slave station IO, index start from 0.
} IOType;

/**
 * @brief DO info
 */
typedef struct {
    IOType io_type;    ///< IO type
    int index;
    BOOL value;
} DOInfo;

/**
* @brief EXtio Data
*/
typedef struct
{
	int din[256];				  ///< Digital input din[0] is the number of valid signals
	int dout[256];				  ///< Digital output dout[0] is the number of valid signals
	float ain[256];				  ///< Analog input din[0] is the number of valid signals
	float aout[256];			  ///< Analog output dout[0] is the number of valid signals
} Io_group;

/**
* @brief Robot joint monitoring data
*/
typedef struct
{
	double instCurrent;		///< Instantaneous current
	double instVoltage;		///< Instantaneous voltage
	double instTemperature; ///< Instantaneous temperature
	double instVel;			///< Instantaneous speed controller 1.7.0.20 and above
	double instTorq;		///< Instantaneous torque
} JointMonitorData;

/**
* @brief Robot monitoring data
*/
typedef struct
{
	double scbMajorVersion;				  ///< scb major version number
	double scbMinorVersion;				  ///< scb minor version number
	double cabTemperature;				  ///< Controller temperature
	double robotAveragePower;			  ///< Robot average voltage
	double robotAverageCurrent;			  ///< Robot average current
	JointMonitorData jointMonitorData[6]; ///< Monitoring data of the robot's six joints
} RobotMonitorData;

/**
* @brief Torque sensor monitoring data
*/
typedef struct
{
	char ip[20];		 ///< Torque sensor IP address
	int port;			 ///< Torque sensor port number
	PayLoad payLoad;	 ///< Tool load
	int status;			 ///< Torque sensor status
	int errcode;		 ///< Torque sensor abnormal error code
	double actTorque[6]; ///< The actual contact force value of the torque sensor (when Initialize is checked) or the raw reading value (when Do Not Initialize is checked)
	double torque[6];	 ///< Torque sensor raw reading value
	double realTorque[6];///< The actual contact force value of the torque sensor (does not change with the initialization options)
} TorqSensorMonitorData;


/**
* @brief Robot status monitoring data, use the get_robot_status function to update the robot status data
*/
typedef struct
{
	int errcode;									///< Error number when the robot runs into an error. 0 means normal operation, and others mean abnormal operation.
	int inpos;										///< The robot movement is in place, 0 means not in place, 1 means in place
	int powered_on;									///< Whether the robot is powered on, 0 means no power, 1 means power on
	int enabled;									///< Flag indicating whether the robot is enabled, 0 means not enabled, 1 means enabled
	double rapidrate;								///< Robot movement ratio
	int protective_stop;							///< Whether the robot detects a collision, 0 means no collision is detected, 1 means a collision is detected
	int emergency_stop;								///< Whether the robot stops suddenly, 0 means no sudden stop, 1 means sudden stop
	int dout[256];									///< Digital output signal of the robot control cabinet, dout[0] is the number of signals
	int din[256];									///< Digital input signal of robot control cabinet, din[0] is the number of signals	
	double ain[256];								///< Robot control cabinet analog input signal, ain[0] is the number of signals
	double aout[256];								///< Robot control cabinet analog output signal, aout[0] is the number of signals
	int tio_dout[16];								///< The digital output signal of the robot end tool, tio_dout[0] is the number of signals
	int tio_din[16];								///< The digital input signal of the robot end tool, tio_din[0] is the number of signals
	double tio_ain[16];								///< Robot end tool analog input signal, tio_ain[0] is the number of signals
	int tio_key[3];                                 ///< Robot end tool buttons [0]free;[1]point;[2]pause_resume;
	Io_group extio;								    ///< Robot external application IO
	Io_group modbus_slave;							///< Robot Modbus Slave
	Io_group profinet_slave;						///< Robot Profinet Slave
	Io_group eip_slave;								///< Robot Ethernet/IP Slave
	unsigned int current_tool_id;					///< The tool coordinate system id currently used by the robot
	double cartesiantran_position[6];				///< The Cartesian space position of the robot end
	double joint_position[6];						///< Robot joint space position
	unsigned int on_soft_limit;						///< Whether the robot is in limit position, 0 means no limit protection is triggered, 1 means limit protection is triggered
	unsigned int current_user_id;					///< The user coordinate system id currently used by the robot
	int drag_status;								///< Whether the robot is in the dragging state, 0 means not in the dragging state, 1 means in the dragging state
	RobotMonitorData robot_monitor_data;			///< Robot status monitoring data
	TorqSensorMonitorData torq_sensor_monitor_data; ///< Robot torque sensor status monitoring data
	int is_socket_connect;							///< Whether the connection channel between SDK and controller is normal, 0 means the connection channel is abnormal, 1 means the connection channel is normal
} RobotStatus;


typedef struct {
    int motion_line;     ///<  motion CMD id
    int motion_line_sdk; ///< reserved
    BOOL inpos;          ///< current motion CMD is done, you should always check queue info at the same time
    int err_add_line;   /// fail to add motion CMD in case robot is already at target position
    int queue;           ///< number of motion CMD in queue
    int active_queue;    ///< number of motion CMD which is under blending
    BOOL queue_full;     ///< cannot push any more motion CMD if queue is full
    BOOL paused;

    BOOL isOnLimit;     ///< soft limit
    BOOL isInEstop;     ///< emergency stop
    BOOL isInCollision; ///< collision
} MotionStatus;

/**
 * @brief config for trajectory record
 */
typedef struct {
    double xyz_interval; ///< cartesian translation acquisition accuracy
    double rpy_interval; ///< cartesian orientation acquisition accuracy
    double vel;          ///< velocity setting for scripty execution
    double acc;          ///< acceleration setting for scripty execution
} TrajTrackPara;

/**
 * @brief admittance config param
 */
typedef struct {
    int axis;
    int opt;             ///< 0: disable, 1: enable
    double ft_user;      ///< the force value to let robot move in MAX velocity, also naming as ft_damping
    double ft_rebound;   ///< rebound force, ability for robot to go back to init position
    double ft_constant;  ///< constant force
    int ft_normal_track; ///< normal vector track，0: disable，1: enable. deprecated, cannot set any more(always disabled).
} AdmitCtrlType;

/**
 * @brief robot admittance control configs collection
 */
typedef struct {
    AdmitCtrlType admit_ctrl[6];
} RobotAdmitCtrl;

/**
 * @brief velocity force control setting
 * there are 3 levels to set，and 1>rate1>rate2>rate3>rate4>0
 * at level 1，able to set rate1,rate2。rate3 and rate4, both are 0
 * at level 2，able to set rate1,rate2，rate3. rate4 is 0
 * at level 3，able to set rate1,rate2，rate3,rate4
 */
typedef struct {
    int vc_level; ///< velocity force control level setting
    double rate1; ///<
    double rate2; ///<
    double rate3; ///<
    double rate4; ///<
} VelCom;

/**
 * @brief force control components
 */
typedef struct {
    double fx; ///< x componet
    double fy; ///< y componet
    double fz; ///< z componet
    double tx; ///< rx componet
    double ty; ///< ry componet
    double tz; ///< rz componet
} FTxyz;

typedef struct {
    double linear[3];
    double angular[3];
} VelocityLimit;
typedef struct {
    int id;           ///< Unique identifier of the sensor (assigned by system)
    int sensor_model; ///< Sensor model ID, 2 for USB sensor, 6 for TIO sensor
    char name[32];    ///< Human-readable sensor name (null-terminated C string)
    int active;       ///< Activation status: 1 if normal, 0 if the sensor is to be deleted
    int status;       ///< Current operational status code of the sensor: 1 if normal, -1 if in error
} FTSensorBasicInfo;

typedef struct {
    int count;  ///< Number of sensors
    FTSensorBasicInfo info[MAX_FTSENSOR_CNT];   ///< Array to hold sensor information
} FTSensorBasicInfoStr;

typedef struct {
    int rule_id;   ///< User-defined rule identifier, can be 0,1,2
    int sensor_id; ///< ID of the associated force-torque sensor
    int active;    ///< Enable flag: 1 for enabled, 0 for disabled
    FTxyz ft;      ///< Force and torque threshold values
} FTSensorSoftLimitRule;

typedef struct {
    FTSensorSoftLimitRule rule[3];   ///< Array to hold sensor information
} FTSensorSoftLimitRuleStr;

typedef struct {
    int count;
    int sensor_id[MAX_FTSENSOR_CNT];
} FTLinkedSensorIDStr;

typedef struct {
    FTxyz ft;
    double min_val;
    double max_val;
} FTSensorThresholdStr;

typedef struct {
    int status;
    FTxyz ft;
} FTSensorDataStr;
typedef struct {
    int sensor_model;    ///< Sensor model identifier
    char name[50];       ///< Name of the sensor
    char comm_param[50]; ///< Communication parameters
} FTSensorConfigStr;

typedef struct {
    double jVel[6]; ///< each joint，unit: rad/s
} JointVelocity;
typedef struct {
    double jtorq[6]; ///< each joint torque，unit：N/m
} JointTorque;
typedef struct
{
    int cab_din[16];
    int cab_dout[16];
    double cab_ain[2];
    double cab_aout[2];
    int tool_din[2];
    int tool_dout[2];
    double tool_ain[2];
} EdgIOState;

typedef struct {
    JointValue jointVal; ///< each joint position，unit：rad
    JointVelocity jointVel; ///< each joint velocity, unit: rad/s
    JointTorque jointTorq; ///< each joint torque，unit：N/m
    CartesianPose cartpose;
    FTxyz torqSensor; 
    EdgIOState IOState;
} EDGState;
/**
 @brief
 */
typedef enum { FTFrame_Tool = 0, FTFrame_World = 1 } FTFrameType;

/**
 *  @brief DH parameters
 */
typedef struct {
    double alpha[6];
    double a[6];
    double d[6];
    double joint_homeoff[6];
} DHParam;

/**
 *  @brief rs485 signal config param
 */
typedef struct {
    char sig_name[20]; ///< signal name
    int chn_id;        ///< RS485 channel id
    int sig_type;      ///< type
    int sig_addr;      ///< address
    int value;         ///<
    int frequency;     ///< must no greater than 20
} SignInfo;

typedef struct {
    char sig_name[20]; ///< signal name
    int frequency;     ///< must no greater than 20
} SignInfo_simple;

/**
 *  @brief rs485 RTU signal config param
 */
typedef struct {
    int chn_id;   ///< RS485 channel ID
    int slaveId;  ///< slave station id, only used with Modbus RTU
    int baudrate; ///< 4800,9600,14400,19200,38400,57600,115200,230400
    int databit;  ///< 7，8
    int stopbit;  ///< 1，2
    int parity;   ///< 78->no check, 79->odd parity check, 69->even parity check
} ModRtuComm;

/**
* @brief basic robot stat
*/
typedef struct
{
	int errcode;	///< 0: normal, others: errorcode
	char errmsg[200]; ///< controller errmsg
	
	int powered_on;	///< 0: power off，1: power on
	int enabled;	///< 0: disabled，1: enabled
} RobotStatus_simple;

/**
* @brief not used
*/
typedef struct
{
	int executingLineId; ///< cmd id
} OptionalCond;

/**
 *@brief 
 */
typedef struct{
	BOOL estoped;     // estop
	BOOL poweredOn;		// power on
	BOOL servoEnabled;	// enable robot or not
}RobotState;

#define MaxLength  256
/**
* @brief 
*/
typedef struct
{
	int len;			             ///< length
	char name[MaxLength][MaxLength]; ///< 
} MultStrStorType;

/**
* @brief tool drive config
*/
typedef struct
{
	int opt;			///< 0: disable, 1: enable
	int axis;			///< axis index, [0,5]
	double rebound;	 	///< rebound ability
	double rigidity;	///< 
} ToolDriveConfig;

/**
* @brief 
*/
typedef struct
{
	ToolDriveConfig config[6];
} RobotToolDriveCtrl;

typedef enum {
    ActualData = 0,
    NormalData = 1,
    RealData = 2
}TorqueDataType;

/**
* @brief torque sensor data
*/
typedef struct
{
	int status;
	int errorCode;
	FTxyz data;
} TorqSensorData;

typedef struct{
    int opt; // 0:disable, 1:enable
    int axis;

    int lower_limit_opt;
    double lower_limit;

    int upper_limit_opt;
    double upper_limit;

} ForceStopCondition;

typedef struct{
    ForceStopCondition condition[6];
} ForceStopConditionList;

/**
* @brief error information
*/
typedef struct
{
	long code;		   ///< error code
	char message[120]; ///< error message
} ErrorCode;

typedef struct
{
	int gain[6]; ///< error message
} DragFrictionCompensationGainList;

/**
* @brief callback
* @param info remember to assign char array, no less than 1024 bytes
				the feedback info contains at least 3 data part: "section", "key", "data"
				which is packed as json. The "data" part represent the current(new) value
*/
typedef void (*CallBackFuncType)(char* info);

#endif
