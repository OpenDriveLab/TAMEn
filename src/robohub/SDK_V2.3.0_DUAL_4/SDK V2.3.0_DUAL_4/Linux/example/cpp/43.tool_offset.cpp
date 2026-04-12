#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "common.h"

#define JK_PI (3.141592653589793)
#define deg_tp_rad 1.0 / 180.0 * JK_PI

int main()
{
    JAKAZuRobot robot;
    RobotStatus robotStatus;
    errno_t ret;

   
    
    ret = robot.login_in("10.5.5.100");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    CartesianPose tool_offsets[2] = {  {7.1,8.1,9,10,11,12} , {1,2,3,4.4,5.4,6}};
    ret = robot.robot_set_tool_offset(LEFT, tool_offsets[0]);
    if (ret == ERR_SUCC)
    {
        printf("Set tool succeed!\n");
    }
    ret = robot.robot_set_tool_offset(RIGHT, tool_offsets[1]);
    if (ret == ERR_SUCC)
    {
        printf("Set tool succeed!\n");
    }


    CartesianPose cur_offset[2];
    ret = robot.robot_get_tool_offset(LEFT, cur_offset);
    ret = robot.robot_get_tool_offset(RIGHT, cur_offset + 1);

    printf("left tool_offsets = %lf, %lf, %lf, %lf, %lf, %lf\n", cur_offset[0].tran.x, cur_offset[0].tran.y, cur_offset[0].tran.z, cur_offset[0].rpy.rx, cur_offset[0].rpy.ry, cur_offset[0].rpy.rz);
    printf("right tool_offsets = %lf, %lf, %lf, %lf, %lf, %lf\n", cur_offset[1].tran.x, cur_offset[1].tran.y, cur_offset[1].tran.z, cur_offset[1].rpy.rx, cur_offset[1].rpy.ry, cur_offset[1].rpy.rz);

    robot.login_out();
    return 0;
}
