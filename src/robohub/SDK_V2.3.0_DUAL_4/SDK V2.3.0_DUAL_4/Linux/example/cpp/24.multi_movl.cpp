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
    // Check that the target poses are valid before use
    JointValue start_pos[2] = { { 0, -10 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, -35 * deg_tp_rad, 0},
                                 { 0, -10 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, 35 * deg_tp_rad, 0} };

    CartesianPose c_pos[2] { { 10, 20, 30, 0, 0, 0 }, { 20, 40, 60, 0, 0, 0 } };
    CartesianPose c_neg[2] { { -10, -20, -30, 0, 0, 0 }, { -20, -40, -60, 0, 0, 0 } };
    
    ret = robot.login_in("127.0.0.1");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    DHParam dh[2] = {};
    ret = robot.robot_get_multi_robot_dh(dh);
    for (int i = 0; i < 2; i++)
    {
        printf("Robot[%d] : Alpha, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].alpha[0], dh[i].alpha[1], dh[i].alpha[2], dh[i].alpha[3], dh[i].alpha[4], dh[i].alpha[5], dh[i].alpha[6]);
        printf("Robot[%d] : d, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].d[0], dh[i].d[1], dh[i].d[2], dh[i].d[3], dh[i].d[4], dh[i].d[5], dh[i].d[6]);
        printf("Robot[%d] : a, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].a[0], dh[i].a[1], dh[i].a[2], dh[i].a[3], dh[i].a[4], dh[i].a[5], dh[i].a[6]);
        printf("Robot[%d] : joint_homeoff, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].joint_homeoff[0], dh[i].joint_homeoff[1], dh[i].joint_homeoff[2], dh[i].joint_homeoff[3], dh[i].joint_homeoff[4], dh[i].joint_homeoff[5], dh[i].joint_homeoff[6]);
    }

    CartesianPose base_offset[2] = {};
    ret = robot.robot_get_default_base(LEFT, base_offset);
    if (ret == ERR_SUCC)
    {
        printf("Left base offset: %lf, %lf, %lf\n", base_offset[0].tran.x, base_offset[0].tran.y, base_offset[0].tran.z);
    }
    ret = robot.robot_get_default_base(RIGHT, base_offset + 1);
    if (ret == ERR_SUCC)
    {
        printf("Right base offset: %lf, %lf, %lf\n", base_offset[1].tran.x, base_offset[1].tran.y, base_offset[1].tran.z);
    }

    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

    ret = robot.enable_robot();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "enable robot");

    {
        MoveMode moveop[2] = {ABS, ABS};
        double vel[2] = {.5, .5};
        double acc[2] = {1, 1};
        ret = robot.robot_run_multi_movj(-1, moveop, TRUE, start_pos, vel, acc);    // Block and wait up to 20 s
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            printf("Error happends when move to joint_pos. %2x\n", ret);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos ok.\n";
        }
    }

    return 0;
    // Blocking mode
    for (int i = 0; i < 5; i++)
    {
        MoveMode moveop[2] = {INCR, INCR};
        double vel[2] = {10, 10};
        double acc[2] = {10, 10};
        ret = robot.robot_run_multi_movl(-1, moveop, TRUE, c_pos, vel, acc);
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos failed.\n";
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos ok.\n";
        }

        ret = robot.robot_run_multi_movl(-1, moveop, TRUE, c_neg, vel, acc);
        if (ret == ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move neg ok.\n";
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move neg failed.\n";
        }
    }

    robot.login_out();
    return 0;
}
