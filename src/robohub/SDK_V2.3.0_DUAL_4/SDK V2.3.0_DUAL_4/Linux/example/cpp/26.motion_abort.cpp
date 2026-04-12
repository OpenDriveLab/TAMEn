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
    errno_t ret = robot.login_in("192.168.2.200");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

    ret = robot.enable_robot();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "enable robot");

    JointValue start_pos[2] = { { 0, -10 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, -35 * deg_tp_rad, 0},
                                 { 0, -10 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, 35 * deg_tp_rad, 0} };
    JointValue end_pos[2] = {};

    {
        MoveMode moveop[2] = {ABS, ABS};
        double vel[2] = {.5, .5};
        double acc[2] = {1, 1};
        ret = robot.robot_run_multi_movj(-1, moveop, FALSE, start_pos, vel, acc);    // Wait up to 20 s in blocking mode
        if (ret != ERR_SUCC)
        {
            printf("Error happends when move to joint_pos. %2x\n", ret);
        }
        else
        {
            while(1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                int inpos[2] = {};
                robot.robot_is_inpos(inpos);
                printf("robot is inpos %d, %d\n", inpos[0], inpos[1]);
                if (inpos[0] && inpos[1])
                {
                    printf("robot is inpos\n");
                    break;
                }
                if (!inpos[0] || !inpos[1])
                {
                    robot.motion_abort();
                }
                if (inpos[0] && inpos[1])
                {
                    printf("robot is inpos\n");
                    break;
                }
            }
        }

        ret = robot.robot_run_multi_movj(-1, moveop, FALSE, end_pos, vel, acc);    // Wait up to 30 s in blocking mode
        if (ret != ERR_SUCC)
        {
            printf("Error happends when move to joint_pos. %2x\n", ret);
        }
        else
        {
            while(1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                int inpos[2] = {};
                robot.robot_is_inpos(inpos);
                printf("robot is inpos %d, %d\n", inpos[0], inpos[1]);
                if (inpos[0] && inpos[1])
                {
                    printf("robot is inpos\n");
                    break;
                }
            }
        }
    }

    robot.login_out();
    return 0;
}
