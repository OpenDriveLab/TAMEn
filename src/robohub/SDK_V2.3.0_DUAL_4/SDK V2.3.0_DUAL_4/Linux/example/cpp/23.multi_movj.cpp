#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "common.h"

#define JK_PI (3.141592653589793)

int main()
{
    JAKAZuRobot robot;
    RobotStatus robotStatus;
    errno_t ret;
    // Check that the target poses are valid before use
    JointValue jstep_pos[2] { { 0.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 }, { 0.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 } };
    JointValue jstep_neg[2] { { -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1 }, { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 } };
    
    ret = robot.login_in("10.5.5.100");
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    // ret = robot.power_on();
    // ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

    ret = robot.disable_robot();

    robot.clear_error();


    ret = robot.enable_robot();
    if (ret != ERR_SUCC)
    {
        printf("enable failed! ret = %d\n", ret);
    }
    // ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "enable robot");

    for (int i = 0; i < 5; i++)
    {
        MoveMode moveop[2] = {INCR, INCR};
        double vel[2] = {1, 1};
        double acc[2] = {1, 1};
        double tol[2] = {0, 0};
        double id[2] = {0, 0};
        ret = robot.robot_run_multi_movj(-1, moveop, FALSE, jstep_pos, vel, acc);
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos failed. ret = " << ret << std::endl;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos ok.\n";
        }

        ret = robot.robot_run_multi_movj(-1, moveop, FALSE, jstep_neg, vel, acc);
        if (ret == ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move neg ok.\n";
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move neg failed. ret = " << ret << std::endl;
        }
    }
    // Blocking mode
    for (int i = 0; i < 5; i++)
    {
        MoveMode moveop[2] = {ABS, ABS};
        double vel[2] = {0.1, 0.1};
        double acc[2] = {0.1, 0.1};
        double tol[2] = {0, 0};
        double id[2] = {0, 0};
        ret = robot.robot_run_multi_movj(-1, moveop, TRUE, jstep_pos, vel, acc);
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

        ret = robot.robot_run_multi_movj(-1, moveop, TRUE, jstep_neg, vel, acc);
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
