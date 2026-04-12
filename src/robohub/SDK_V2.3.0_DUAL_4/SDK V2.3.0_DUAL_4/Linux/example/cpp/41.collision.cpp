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


    for (int i = 0; i < 6; i++)
    {
        ret = robot.set_collision_level(LEFT, i);
        if (ret == ERR_SUCC)
        {
            printf("set collision level succeed\n");
        }
         std::this_thread::sleep_for(std::chrono::seconds(1));
        int level = -1;
        ret = robot.get_collision_level(LEFT, &level);
        if (ret == ERR_SUCC)
        {
            printf("get collision level succeed, level: %d\n", level);
        }
    }

    for (int i = 5; i > -1; i--)
    {
        ret = robot.set_collision_level(LEFT, i);
        if (ret == ERR_SUCC)
        {
            printf("set collision level succeed\n");
        }
         std::this_thread::sleep_for(std::chrono::seconds(1));
        int level = -1;
        ret = robot.get_collision_level(LEFT, &level);
        if (ret == ERR_SUCC)
        {
            printf("get collision level succeed, level: %d\n", level);
        }
    }

    robot.login_out();
    return 0;
}
