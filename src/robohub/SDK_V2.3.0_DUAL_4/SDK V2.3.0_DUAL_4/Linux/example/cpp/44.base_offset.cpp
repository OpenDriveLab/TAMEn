#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "common.h"

#define JK_PI (3.141592653589793)
#define deg_tp_rad 1.0 / 180.0 * JK_PI

int main(){
    JAKAZuRobot robot;
    errno_t ret;

    robot.login_in("10.5.5.100");
      
    ret = robot.power_on();

    CartesianPose base_offset[2] = {};
    
    ret = robot.robot_get_default_base(LEFT, base_offset);
    if (ret == ERR_SUCC)
    {
        printf("Left base offset: %lf, %lf, %lf, %lf, %lf, %lf\n", base_offset[0].tran.x, base_offset[0].tran.y, base_offset[0].tran.z, base_offset[0].rpy.rx, base_offset[0].rpy.ry, base_offset[0].rpy.rz);
    }
    else
    {
        printf("Failed to get left base offset: %d\n", ret);
        return -1;
    }
    
    ret = robot.robot_get_default_base(RIGHT, base_offset + 1);
    if (ret == ERR_SUCC)
    {
        printf("Right base offset: %lf, %lf, %lf, %lf, %lf, %lf\n", base_offset[1].tran.x, base_offset[1].tran.y, base_offset[1].tran.z, base_offset[1].rpy.rx, base_offset[1].rpy.ry, base_offset[1].rpy.rz);
    }
    else
    {
        printf("Failed to get right base offset: %d\n", ret);
        return -1;
    }


    robot.login_out();
    return 0;
}