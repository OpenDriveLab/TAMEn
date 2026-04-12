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
    JointValue start_pos[2] = { { 0, 90 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, -35 * deg_tp_rad, 0},
                                 { 0, -90 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, 35 * deg_tp_rad, 0} };

    CartesianPose c_pos[2] { { 0, 0, 30, 0, 0, 0 }, { 0, 0, 60, 0, 0, 0 } };
    
    ret = robot.login_in("10.5.5.100");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");


    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");
    printf("power on succeed\n");

    ret = robot.enable_robot();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "enable robot");
    printf("enable succeed\n");

    ret = robot.set_collision_level(LEFT, 3);
    if (ret == ERR_SUCC)
    {
        printf("set collision level succeed\n");
    }

    while(1)
    {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        int error[2] = {-1, -1};
        ret = robot.robot_is_in_error(error);
        if (ret == ERR_SUCC)
        {
            printf("robot has error %s, %s\n", error[0] ? "true" : "false", error[1] ? "true" : "false");
        }
        ErrorCode code;
        ret = robot.get_last_error(&code);
        if (code.code != 0)
        {
            printf("errcode = %lx, %s\n", code.code, code.message);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            robot.clear_error();
        }
        else
        {
            printf("no error\n");
        }
    }

    robot.login_out();
    return 0;
}
